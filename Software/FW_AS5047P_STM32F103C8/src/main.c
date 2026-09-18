/**
 * @file  main.c
 * @brief Опрос энкодера AS5047P по SPI с фильтрацией угла следящим
 *        наблюдателем 2-го порядка (см. angle_filter.h) и выводом в лог.
 *
 * Датчик: плата AS5047P-TS_EK_AB, гребёнка UP1: 5V GND 3V3 CSn CLK MOSI MISO GND.
 * ВНИМАНИЕ: перемычка JP1 на плате датчика выбирает питание 5V / 3V3.
 *           Ставим JP1 в положение 3V3 и питаем от 3.3 В — тогда и уровни SPI
 *           будут 3.3 В, без согласования с STM32.
 *
 * МК: WeAct Studio STM32F1 Core Board (STM32F103C8T6, HSE 8 МГц).
 *
 *   AS5047P EK        STM32F103C8T6
 *   ----------        -------------
 *   3V3         <->   3V3
 *   GND         <->   G (GND)
 *   CSn         <->   PA4   (GPIO, программный NSS)
 *   CLK         <->   PA5   (SPI1_SCK)
 *   MISO        <->   PA6   (SPI1_MISO)
 *   MOSI        <->   PA7   (SPI1_MOSI)
 *
 *   Лог:        USB (разъём USB-C платы) — виртуальный COM-порт, класс CDC.
 *   Светодиод:  PB2 (WeAct). На классическом Blue Pill — PC13, см. LED_PORT.
 *
 * Структура: ВСЕ обращения к датчику идут из прерывания TIM2 с постоянным
 * периодом — фильтру нужен строго равномерный шаг Ts, а печать в USB CDC
 * блокирующая и дала бы джиттер в десятки миллисекунд. Главный цикл только
 * снимает копию состояния и печатает.
 */
#include <stdio.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "as5047p.h"
#include "angle_filter.h"
#if !LOG_USE_UART
#include "usb_device.h"
#include "usbd_cdc_if.h"
#endif

/* --- Конфигурация (можно переопределить через build_flags) --- */
#ifndef SAMPLE_RATE_HZ
#define SAMPLE_RATE_HZ   2000U  /* частота опроса датчика, Гц */
#endif
#ifndef FILTER_BW_HZ
#define FILTER_BW_HZ     10.0f  /* полоса следящего контура, Гц */
#endif
#ifndef FILTER_DAMPING
#define FILTER_DAMPING   1.0f   /* zeta: 1.0 — критическое демпфирование */
#endif
#ifndef LOG_PLOT
#define LOG_PLOT         1      /* 1 — Arduino Serial Plotter, 0 — текстовый лог */
#endif

#if LOG_PLOT
#define PRINT_PERIOD_MS  20U    /* 50 Гц — график в Serial Plotter без пропусков */
#else
#define PRINT_PERIOD_MS  100U   /* период печати строки измерения */
#define DIAG_PERIOD_MS   2000U  /* период печати диагностики (DIAAGC/MAG) */
#endif

/* Гистерезис печати flt: не менять десятую, пока угол не ушёл дальше
   чем 0.05° (полшага) + 0.04°. Иначе остаточный шум контура (~0.014° СКО)
   на границе 180.05 даёт пилу 180.0↔180.1 — это не датчик, а квантование. */
#define PLOT_HYST_CDEG   4

#define ENC_CS_PORT      GPIOA
#define ENC_CS_PIN       GPIO_PIN_4

/* WeAct STM32F1 Core Board: LED на PB2. Классический Blue Pill: GPIOC / GPIO_PIN_13. */
#define LED_PORT         GPIOB
#define LED_PIN          GPIO_PIN_2

SPI_HandleTypeDef  hspi1;
TIM_HandleTypeDef  htim2;
#if LOG_USE_UART
UART_HandleTypeDef huart1;
#endif

static as5047p_t      encoder;
static angle_filter_t filter;
static char           stdout_buf[192];  /* построчная буферизация: строка = посылка */

/**
 * Статистика за окно печати. Заполняется в прерывании, снимается главным
 * циклом. Разбросы считаются относительно опорной точки окна, чтобы переход
 * через 0/360 градусов не портил размах.
 */
typedef struct {
    uint32_t n;             /* принятых отсчётов */
    uint32_t miss;          /* тактов без измерения (сбой обмена) */
    uint32_t gated;         /* поправок, обрезанных ограничителем */
    int32_t  raw_min;       /* размах СЫРОГО угла, отсчёты */
    int32_t  raw_max;
    int32_t  flt_min;       /* размах ФИЛЬТРОВАННОГО угла, отсчёты */
    int32_t  flt_max;
    uint64_t sum_e2;        /* сумма квадратов невязки, отсчёты^2 */
    float    ref;           /* опорная точка окна, отсчёты */
    uint8_t  have_ref;
} win_stats_t;

static win_stats_t stats;   /* пишется только из ISR, читается с закрытыми IRQ */

/* Обмен с датчиком идёт только из ISR, поэтому диагностические регистры
   читаются по заявке из главного цикла, а не напрямую. */
static volatile uint8_t  diag_request;
static volatile uint8_t  diag_ready;
static volatile uint16_t diag_diaagc;
static volatile uint16_t diag_mag;
static volatile uint8_t  diag_status;
static volatile uint16_t last_errfl;
static volatile uint8_t  last_read_status;
static volatile uint16_t last_raw;          /* последний сырой угол, для графика */
#if LOG_PLOT
static int32_t plot_flt_tenth;              /* последняя напечатанная десятая flt */
static uint8_t plot_flt_have;
#endif

/* Длительность такта опроса по счётчику тактов ядра (DWT): подтверждает,
   что обмен + фильтр укладываются в период дискретизации. */
static volatile uint32_t isr_cycles_max;
static volatile uint32_t isr_cycles_sum;
static volatile uint32_t isr_cycles_n;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
#if LOG_USE_UART
static void MX_USART1_UART_Init(void);
#endif
#if !LOG_PLOT
static void print_header(void);
static void print_diagnostics(void);
static uint32_t isqrt64(uint64_t v);
#endif
#if LOG_PLOT
static void print_tenth(int32_t t);
static int32_t to_tenth(int32_t x100);
static int32_t sticky_tenth(int32_t x100, int32_t *last, uint8_t *have);
#endif
static int32_t unwrapped_cdeg(float theta, int32_t turns);
static int32_t wrap_round(float d);
static void stats_reset(void);
static void cycle_counter_init(void);
void Error_Handler(void);

int main(void)
{
    uint32_t next_print = 0;
#if !LOG_PLOT
    uint32_t next_diag = 0;
#endif

    HAL_Init();
    SystemClock_Config();

    /* SysTick — выше TIM2 (приоритет 1) и USB (5): иначе таймауты HAL внутри
       прерывания опроса зависли бы, т.к. HAL_GetTick() перестал бы расти. */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    cycle_counter_init();

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
#if LOG_USE_UART
    MX_USART1_UART_Init();
#else
    MX_USB_DEVICE_Init();
#endif

    /* _IOLBF: printf копит строку и отдаёт её в _write целиком по переводу
       строки. Для USB это принципиально — иначе каждый символ уходил бы
       отдельным пакетом. */
    setvbuf(stdout, stdout_buf, _IOLBF, sizeof(stdout_buf));

#if !LOG_USE_UART
    /* даём хосту время на перечисление, иначе шапка уйдёт в никуда */
    (void)usb_device_wait_ready(3000U);
    HAL_Delay(200);
#endif

    encoder.hspi    = &hspi1;
    encoder.cs_port = ENC_CS_PORT;
    encoder.cs_pin  = ENC_CS_PIN;

    angle_filter_init(&filter, (float)SAMPLE_RATE_HZ, FILTER_BW_HZ, FILTER_DAMPING);
    stats_reset();

    HAL_Delay(10);                                          /* пауза после подачи питания */
    (void)as5047p_read(&encoder, AS5047P_REG_ERRFL, NULL);  /* холостое чтение: чистим ERRFL */

#if !LOG_PLOT
    print_header();
    print_diagnostics();
#endif

    /* с этого момента SPI принадлежит прерыванию TIM2 */
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    for (;;) {
        uint32_t now = HAL_GetTick();

        if ((int32_t)(now - next_print) >= 0) {
            angle_filter_t snap;
            uint16_t       raw;
            int32_t        flt_cdeg;
            int32_t        raw_cdeg;
#if !LOG_PLOT
            int32_t        cdeg_s;
            win_stats_t    w;
            uint32_t       rms_x100 = 0;
            uint32_t       w_abs;
            char           sign;
#endif

            next_print = now + PRINT_PERIOD_MS;

            /* Снимок согласованного состояния: фильтр и статистика меняются
               в прерывании, читать их по частям нельзя. Под запретом прерываний
               делаем только копирование — пересчёт в градусы (программная
               плавающая точка) уже на копии, чтобы не добавлять джиттер такту. */
            __disable_irq();
#if !LOG_PLOT
            w = stats;
#endif
            snap = filter;
            raw  = last_raw;
            stats_reset();
            __enable_irq();

            /* В сотые доли из float, без округления theta до целого отсчёта:
               иначе ±0.5 count (±0.011°) лишний раз толкает значение к границе. */
            {
                float e = (float)raw - snap.theta;

                if (e >= (float)ANGLE_FILTER_HALF) {
                    e -= (float)ANGLE_FILTER_COUNTS;
                } else if (e < -(float)ANGLE_FILTER_HALF) {
                    e += (float)ANGLE_FILTER_COUNTS;
                }
                flt_cdeg = unwrapped_cdeg(snap.theta, snap.turns);
                raw_cdeg = unwrapped_cdeg(snap.theta + e, snap.turns);
            }
#if !LOG_PLOT
            cdeg_s   = angle_filter_centideg_per_s(&snap);
#endif

#if LOG_PLOT
            /* Arduino Serial Plotter: label:value через запятую, без текста.
               Угол развёрнут (без скачка 360→0), чтобы график не рвался.
               raw — ближайшая десятая (виден шум датчика);
               flt — с гистерезисом, чтобы округление не пилило на границе. */
            printf("raw:");
            print_tenth(to_tenth(raw_cdeg));
            printf(",flt:");
            print_tenth(sticky_tenth(flt_cdeg, &plot_flt_tenth, &plot_flt_have));
            printf("\r\n");
#else
            if (w.n > 0U) {
                /* СКО невязки (raw - прогноз) ~ шум самого датчика: прогноз
                   почти не шумит, полоса контура много уже частоты опроса.
                   Забракованные отсчёты сюда не попадают. */
                rms_x100 = isqrt64((w.sum_e2 * 10000ULL) / (uint64_t)w.n);
            }

            sign  = (cdeg_s < 0) ? '-' : '+';
            w_abs = (uint32_t)((cdeg_s < 0) ? -cdeg_s : cdeg_s);

            printf("[%8lu ms] raw=%3ld.%02lu flt=%3ld.%02lu deg  w=%c%lu.%02lu deg/s  "
                   "p-p raw/flt=%ld/%ld  rms=%lu.%02lu  n=%lu miss=%lu gate=%lu\r\n",
                   (unsigned long)now,
                   (long)(raw_cdeg / 100), (unsigned long)((uint32_t)((raw_cdeg < 0 ? -raw_cdeg : raw_cdeg) % 100)),
                   (long)(flt_cdeg / 100), (unsigned long)((uint32_t)((flt_cdeg < 0 ? -flt_cdeg : flt_cdeg) % 100)),
                   sign, (unsigned long)(w_abs / 100UL), (unsigned long)(w_abs % 100UL),
                   (long)(w.n ? (w.raw_max - w.raw_min) : 0),
                   (long)(w.n ? (w.flt_max - w.flt_min) : 0),
                   (unsigned long)(rms_x100 / 100UL), (unsigned long)(rms_x100 % 100UL),
                   (unsigned long)w.n, (unsigned long)w.miss, (unsigned long)w.gated);
#endif

            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        }

#if !LOG_PLOT
        if ((int32_t)(now - next_diag) >= 0) {
            next_diag = now + DIAG_PERIOD_MS;
            print_diagnostics();
        }
#endif
    }
}

/* ------------------------------------------------------------------------ */
/*  Опрос датчика: строго равномерный такт                                   */
/* ------------------------------------------------------------------------ */

#if LOG_PLOT
/** Печать уже готовой десятой доли градуса: 1801 → 180.1 */
static void print_tenth(int32_t t)
{
    uint32_t a;

    if (t < 0) {
        a = (uint32_t)(-t);
        printf("-%lu.%lu", (unsigned long)(a / 10UL), (unsigned long)(a % 10UL));
    } else {
        a = (uint32_t)t;
        printf("%lu.%lu", (unsigned long)(a / 10UL), (unsigned long)(a % 10UL));
    }
}

/** Сотые → ближайшая десятая. */
static int32_t to_tenth(int32_t x100)
{
    return (x100 >= 0) ? (x100 + 5) / 10 : (x100 - 5) / 10;
}

/**
 * Округление с гистерезисом: пока значение держится около уже напечатанной
 * десятой, не переключаемся. Смена — только если ушли дальше чем
 * полшага (0.05°) плюс PLOT_HYST_CDEG.
 */
static int32_t sticky_tenth(int32_t x100, int32_t *last, uint8_t *have)
{
    int32_t nearest = to_tenth(x100);
    int32_t d;

    if (*have == 0U) {
        *last = nearest;
        *have = 1U;
        return nearest;
    }
    d = x100 - (*last * 10);
    if (d > (5 + PLOT_HYST_CDEG) || d < -(5 + PLOT_HYST_CDEG)) {
        *last = nearest;
    }
    return *last;
}
#endif /* LOG_PLOT */

/** Развёрнутый угол в сотых градуса прямо из float, без округления до отсчёта. */
static int32_t unwrapped_cdeg(float theta, int32_t turns)
{
    float c = ((float)turns * (float)ANGLE_FILTER_COUNTS + theta) * 2.197265625f;
    return (int32_t)(c >= 0.0f ? c + 0.5f : c - 0.5f);
}

/** Приведение разности углов к диапазону +-полоборота, в отсчётах. */
static int32_t wrap_round(float d)
{
    if (d >= (float)ANGLE_FILTER_HALF) {
        d -= (float)ANGLE_FILTER_COUNTS;
    } else if (d < -(float)ANGLE_FILTER_HALF) {
        d += (float)ANGLE_FILTER_COUNTS;
    }
    return (int32_t)(d >= 0.0f ? d + 0.5f : d - 0.5f);
}

/** Учёт отсчёта в статистике окна (вызывается из ISR после обновления фильтра). */
static void stats_account(uint16_t raw)
{
    int32_t di;

    if (!stats.have_ref) {
        stats.ref      = filter.theta;
        stats.have_ref = 1U;
        stats.raw_min  = 0;
        stats.raw_max  = 0;
        stats.flt_min  = 0;
        stats.flt_max  = 0;
    }

    /* отклонение сырого отсчёта от опорной точки, с учётом перехода через оборот */
    di = wrap_round((float)raw - stats.ref);
    if (di < stats.raw_min) {
        stats.raw_min = di;
    }
    if (di > stats.raw_max) {
        stats.raw_max = di;
    }

    /* то же для оценки фильтра */
    di = wrap_round(filter.theta - stats.ref);
    if (di < stats.flt_min) {
        stats.flt_min = di;
    }
    if (di > stats.flt_max) {
        stats.flt_max = di;
    }

    stats.sum_e2 += (uint64_t)((int64_t)filter.last_err * (int64_t)filter.last_err);
    stats.n++;
}

/** Счётчик тактов ядра Cortex-M3: нужен только для профилирования такта. */
static void cycle_counter_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/** Один такт опроса: чтение угла, шаг фильтра, при заявке — диагностика. */
static void sample_tick(void)
{
    uint32_t t0 = DWT->CYCCNT;
    uint16_t raw = 0;
    as5047p_status_t st;
    bool served_diag = false;

    st = as5047p_read_angle(&encoder, &raw);
    if (st == AS5047P_OK) {
        uint32_t gated_before = filter.n_gated;

        last_raw = raw;
        angle_filter_update(&filter, raw);
        if (filter.n_gated != gated_before) {
            /* Отсчёт забракован как выброс — в оценку шума он не идёт,
               иначе одна помеха испортила бы rms за всё окно. */
            stats.gated++;
        } else if (filter.locked) {
            stats_account(raw);
        }
    } else {
        uint16_t errfl = 0;

        /* Отсчёт потерян: ведём угол по модели, а не повторяем прошлое значение —
           повтор дал бы фильтру ложную «остановку». */
        angle_filter_predict(&filter);
        last_read_status = (uint8_t)st;
        stats.miss++;

        if (as5047p_read(&encoder, AS5047P_REG_ERRFL, &errfl) == AS5047P_OK) {
            last_errfl = errfl;
        }
    }

    if (diag_request && !diag_ready) {
        uint16_t v = 0;

        diag_status = (uint8_t)as5047p_read(&encoder, AS5047P_REG_DIAAGC, &v);
        diag_diaagc = v;
        v = 0;
        (void)as5047p_read(&encoder, AS5047P_REG_MAG, &v);
        diag_mag     = v;
        diag_request = 0U;
        diag_ready   = 1U;
        served_diag  = true;
    }

    /* В статистику берём только обычные такты: такт с диагностикой делает
       втрое больше обменов и не отражает установившуюся нагрузку. */
    if (!served_diag) {
        uint32_t dt = DWT->CYCCNT - t0;

        if (dt > isr_cycles_max) {
            isr_cycles_max = dt;
        }
        isr_cycles_sum += dt;
        isr_cycles_n++;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        sample_tick();
    }
}

static void stats_reset(void)
{
    stats.n        = 0U;
    stats.miss     = 0U;
    stats.gated    = 0U;
    stats.raw_min  = 0;
    stats.raw_max  = 0;
    stats.flt_min  = 0;
    stats.flt_max  = 0;
    stats.sum_e2   = 0U;
    stats.have_ref = 0U;
}

#if !LOG_PLOT
/** Целочисленный квадратный корень (побитовый, «в столбик»), без math.h. */
static uint32_t isqrt64(uint64_t v)
{
    uint64_t rem = 0;
    uint64_t root = 0;
    int i;

    for (i = 0; i < 32; i++) {
        root <<= 1;
        rem = (rem << 2) | (v >> 62);
        v <<= 2;
        if (root < rem) {
            root++;
            rem -= root;
            root++;
        }
    }
    return (uint32_t)(root >> 1);
}
#endif /* !LOG_PLOT */

/* ------------------------------------------------------------------------ */
/*  Печать                                                                   */
/* ------------------------------------------------------------------------ */

#if !LOG_PLOT
/** Шапка: параметры такта и настройки фильтра — чтобы лог был самодостаточен. */
static void print_header(void)
{
    float    bn         = angle_filter_noise_bandwidth(&filter);
    uint32_t bw_x100    = (uint32_t)(filter.bandwidth_hz * 100.0f + 0.5f);
    uint32_t zeta_x100  = (uint32_t)(filter.damping * 100.0f + 0.5f);
    uint32_t bn_x100    = (uint32_t)(bn * 100.0f + 0.5f);
    uint32_t bwmax_x100 = (uint32_t)(angle_filter_max_bandwidth((float)SAMPLE_RATE_HZ) * 100.0f + 0.5f);
    uint32_t kp_x1000   = (uint32_t)(filter.gain_p * 1000.0f + 0.5f);
    uint32_t ki_x1000   = (uint32_t)(filter.gain_i * 1000.0f + 0.5f);
    /* ожидаемое подавление белого шума измерения: sqrt(f_s / (2*Bn)) раз */
    uint32_t supp_x100  = (bn > 0.0f)
                        ? (uint32_t)(isqrt64((uint64_t)((float)SAMPLE_RATE_HZ / (2.0f * bn) * 10000.0f)))
                        : 0U;

    printf("\r\n\r\n=== AS5047P + angle tracking observer ===\r\n");
    printf("SYSCLK %lu Hz, SPI1 %lu Hz, sample %lu Hz\r\n",
           (unsigned long)HAL_RCC_GetHCLKFreq(),
           (unsigned long)(HAL_RCC_GetPCLK2Freq() / 32UL),
           (unsigned long)SAMPLE_RATE_HZ);
    printf("filter: type-II PLL, f_bw=%lu.%02lu Hz (max %lu.%02lu), zeta=%lu.%02lu\r\n",
           (unsigned long)(bw_x100 / 100UL), (unsigned long)(bw_x100 % 100UL),
           (unsigned long)(bwmax_x100 / 100UL), (unsigned long)(bwmax_x100 % 100UL),
           (unsigned long)(zeta_x100 / 100UL), (unsigned long)(zeta_x100 % 100UL));
    printf("        Kp*Ts=%lu.%03lu  Ki*Ts=%lu.%03lu 1/s  Bn=%lu.%02lu Hz  "
           "expected noise /%lu.%02lu\r\n",
           (unsigned long)(kp_x1000 / 1000UL), (unsigned long)(kp_x1000 % 1000UL),
           (unsigned long)(ki_x1000 / 1000UL), (unsigned long)(ki_x1000 % 1000UL),
           (unsigned long)(bn_x100 / 100UL), (unsigned long)(bn_x100 % 100UL),
           (unsigned long)(supp_x100 / 100UL), (unsigned long)(supp_x100 % 100UL));
    printf("columns: flt = filtered angle, w = estimated speed, p-p = peak-to-peak "
           "over window, rms = residual (1 count = 0.022 deg)\r\n");
}

/** Состояние магнита и АРУ: сразу видно, правильно ли выставлен магнит. */
static void print_diagnostics(void)
{
    uint16_t diaagc = 0;
    uint16_t mag = 0;

    /* Заявка обслуживается прерыванием опроса; если таймер ещё не пущен,
       читаем сами. */
    if ((htim2.Instance->CR1 & TIM_CR1_CEN) == 0U) {
        as5047p_status_t s = as5047p_read(&encoder, AS5047P_REG_DIAAGC, &diaagc);

        if (s != AS5047P_OK) {
            printf("    DIAG: read failed (%s)\r\n", as5047p_strerror(s));
            return;
        }
        (void)as5047p_read(&encoder, AS5047P_REG_MAG, &mag);
    } else {
        uint32_t started;

        diag_ready   = 0U;
        diag_request = 1U;

        started = HAL_GetTick();
        while (!diag_ready) {
            if ((HAL_GetTick() - started) > 100U) {
                printf("    DIAG: no answer from sampler\r\n");
                diag_request = 0U;
                return;
            }
        }
        if (diag_status != (uint8_t)AS5047P_OK) {
            printf("    DIAG: read failed (%s)\r\n",
                   as5047p_strerror((as5047p_status_t)diag_status));
            return;
        }
        diaagc = diag_diaagc;
        mag    = diag_mag;
    }

    printf("    DIAG: AGC=%3u  MAG=%5u  %s%s%s%s turns=%ld relock=%lu\r\n",
           (unsigned)(diaagc & AS5047P_DIAAGC_AGC_Msk), (unsigned)mag,
           (diaagc & AS5047P_DIAAGC_LF)   ? "LF "        : "offset-not-ready ",
           (diaagc & AS5047P_DIAAGC_COF)  ? "COF! "      : "",
           (diaagc & AS5047P_DIAAGC_MAGH) ? "MAG-HIGH! " : "",
           (diaagc & AS5047P_DIAAGC_MAGL) ? "MAG-LOW! "  : "",
           (long)filter.turns, (unsigned long)filter.n_relock);

    {
        uint32_t cmax, csum, cn;

        __disable_irq();
        cmax = isr_cycles_max;
        csum = isr_cycles_sum;
        cn   = isr_cycles_n;
        isr_cycles_max = 0U;
        isr_cycles_sum = 0U;
        isr_cycles_n   = 0U;
        __enable_irq();

        if (cn > 0U) {
            uint32_t mhz  = HAL_RCC_GetHCLKFreq() / 1000000UL;
            uint32_t avg  = csum / cn;
            /* загрузка ядра тактом опроса, сотые доли процента */
            uint32_t load = (uint32_t)(((uint64_t)avg * SAMPLE_RATE_HZ * 10000ULL) /
                                       HAL_RCC_GetHCLKFreq());

            printf("    TICK: avg %lu.%02lu us, max %lu.%02lu us, period %lu us, "
                   "cpu %lu.%02lu%%\r\n",
                   (unsigned long)(avg / mhz), (unsigned long)((avg % mhz) * 100UL / mhz),
                   (unsigned long)(cmax / mhz), (unsigned long)((cmax % mhz) * 100UL / mhz),
                   (unsigned long)(1000000UL / SAMPLE_RATE_HZ),
                   (unsigned long)(load / 100UL), (unsigned long)(load % 100UL));
        }
    }

    if (last_read_status != (uint8_t)AS5047P_OK) {
        printf("    LAST ERROR: %s  ERRFL=0x%04X%s%s%s\r\n",
               as5047p_strerror((as5047p_status_t)last_read_status),
               (unsigned)last_errfl,
               (last_errfl & AS5047P_ERRFL_FRERR)   ? " FRERR"   : "",
               (last_errfl & AS5047P_ERRFL_INVCOMM) ? " INVCOMM" : "",
               (last_errfl & AS5047P_ERRFL_PARERR)  ? " PARERR"  : "");
        last_read_status = (uint8_t)AS5047P_OK;
    }
}
#endif /* !LOG_PLOT */

/* ------------------------------------------------------------------------ */
/*  Инициализация периферии                                                  */
/* ------------------------------------------------------------------------ */

/**
 * HSE 8 МГц -> PLL x9 -> SYSCLK 72 МГц, APB1 36 МГц, APB2 72 МГц.
 * Для USB обязателен ровно 48 МГц: 72 / 1.5 = 48.
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
#if !LOG_USE_UART
    RCC_PeriphCLKInitTypeDef periph = {0};
#endif

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

#if !LOG_USE_UART
    periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
    periph.UsbClockSelection    = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) {
        Error_Handler();
    }
#endif
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* CSn выставляем в неактивный уровень ДО перевода вывода в выход */
    HAL_GPIO_WritePin(ENC_CS_PORT, ENC_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    gpio.Pin   = ENC_CS_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ENC_CS_PORT, &gpio);

    gpio.Pin   = LED_PIN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);
}

/** SPI1 master, 16 бит, режим 1 (CPOL=0, CPHA=1), 72/32 = 2.25 МГц. */
static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_16BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;    /* CPOL = 0 */
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;     /* CPHA = 1 */
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * TIM2 — такт опроса. На APB1 с делителем 2 таймеры тактируются удвоенной
 * частотой шины: 2 * 36 МГц = 72 МГц. Предделитель 72 даёт счёт в микросекундах.
 */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  src = {0};
    TIM_MasterConfigTypeDef mc  = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 72U - 1U;                        /* -> 1 МГц */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = (1000000U / SAMPLE_RATE_HZ) - 1U;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    src.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &src) != HAL_OK) {
        Error_Handler();
    }

    mc.MasterOutputTrigger = TIM_TRGO_RESET;
    mc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &mc) != HAL_OK) {
        Error_Handler();
    }
}

#if LOG_USE_UART
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}
#endif /* LOG_USE_UART */

void Error_Handler(void)
{
    __disable_irq();
    for (;;) {
        /* останов: смотреть отладчиком */
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("ASSERT FAILED: %s:%lu\r\n", (const char *)file, (unsigned long)line);
}
#endif
