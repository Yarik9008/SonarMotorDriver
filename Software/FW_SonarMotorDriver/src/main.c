/**
 * @file main.c
 * @brief Основной модуль приложения. Координация работы всех подсистем.
 *
 * Модуль реализует:
 * - Инициализацию аппаратной части и программных модулей (неблокирующая стейт-машина).
 * - Стартовую диагностику энкодера ДО разрешения движения (серия чтений BiSS-C:
 *   доля валидных ответов + разброс позиции; при провале мотор остаётся выключенным,
 *   хост получает err:enc, при успехе — enc:ok и штатный выход в «дом»).
 * - Главный цикл управления (Control Loop, 1 кГц по TIM2): опрос энкодера, PID, мотор.
 * - STEP/DIR: непрерывная частота (TIM4 PWM), без перезапуска каждый тик — нет рывков.
 * - Защиту от блокировки вала: команда есть, вал по энкодеру стоит дольше
 *   STALL_TIMEOUT_MS — драйвер выключается, хост получает err:stall.
 * - Обработку команд пользователя через UART.
 * - Периодический вывод телеметрии.
 * - Логику автоматического сканирования секторов.
 */

#include "stm32f1xx_hal.h"
#include "board.h"
#include "biss_c.h"
#include "tmc2209/tmc2209_motor.h"
#include "tmc2209/tmc2209_port_stm32_hal.h"
#include "pid.h"
#include "cmd_parser.h"
#include "uart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

/* --- Прототипы --- */

static void SystemClock_Config(void);
static void BSP_Init(void);
static void PollTimer_Init(void);
static void Sample_ISR(void);
static void ProcessCommand(const Cmd_Result *cmd);
static void SendResponse(const char *fmt, ...);
static void PollCommands(void);
static void Encoder_Accumulate(const BiSS_Reading *rd);
static uint8_t Encoder_FilterOutlier(const BiSS_Reading *rd);
static void Mode_Update(uint8_t enc_ok);
static void MotorControl_Tick(uint8_t enc_ok);
static void Stall_Tick(uint8_t enc_ok);
static void Telemetry_Tick(uint8_t enc_ok, BiSS_Status st);
static void Heartbeat_Tick(void);
static void Scan_Tick(void);

/* --- Константы --- */

#define DEG_PER_STEP_DEFAULT (360.0f / (float)MOTOR_STEPS_PER_REV)
#define MAX_DEG_PER_TICK ((float)MAX_SPEED_DEG_S / (float)POLL_FREQ_HZ)
#define DT_S             (1.0f / (float)POLL_FREQ_HZ)
#define COUNTS_TO_DEG(c) ((float)(c) * 360.0f / (float)ENCODER_COUNTS_REV)
/* Double-версия: для абсолютной позиции (g_enc_counts до ~3.6e9 отсчётов
 * теряет во float до ~0.7°) и накопителей цели — сужаем во float только
 * малые разности (ошибку контура). */
#define COUNTS_TO_DEG_D(c) ((double)(c) * 360.0 / (double)ENCODER_COUNTS_REV)

/* Тиков опроса до принудительного прерывания зависшего DMA-чтения BiSS */
#define BISS_DMA_TIMEOUT_TICKS \
    ((BISS_DMA_TIMEOUT_MS + POLL_INTERVAL_MS - 1U) / POLL_INTERVAL_MS)

/* Тиков коастинга в CL без данных энкодера до остановки мотора */
#define ENCODER_COAST_TICKS  (ENCODER_COAST_MS / POLL_INTERVAL_MS)

/* Окна детектора блокировки вала в тиках опроса */
#define STALL_TIMEOUT_TICKS    (STALL_TIMEOUT_MS / POLL_INTERVAL_MS)
#define STALL_FAST_TICKS       (STALL_FAST_MS / POLL_INTERVAL_MS)
#define STALL_IDLE_RESET_TICKS (STALL_IDLE_RESET_MS / POLL_INTERVAL_MS)

/* Реальный масштаб градус↔шаг: следует за командой mstep (микрошаг драйвера
 * меняется в рантайме, а DEG_PER_STEP_DEFAULT — только compile-time значение). */
static float g_deg_per_step = DEG_PER_STEP_DEFAULT;

static inline int32_t DegToSteps(float deg)
{ return (int32_t)(deg / g_deg_per_step); }

static inline float clampf(float v, float lo, float hi)
{ return (v < lo) ? lo : (v > hi) ? hi : v; }

static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi)
{ return (v < lo) ? lo : (v > hi) ? hi : v; }

static inline float shortest_path_err(float target, float pos)
{
    float err = target - pos;
    while (err > 180.0f)   err -= 360.0f;
    while (err <= -180.0f) err += 360.0f;
    return err;
}

/* Разность позиций энкодера a - b (в отсчётах) с учётом перехода через ноль */
static inline int32_t enc_wrap_delta(uint32_t a, uint32_t b)
{
    int32_t d = (int32_t)a - (int32_t)b;
    if (d >  (int32_t)(ENCODER_COUNTS_REV / 2)) d -= (int32_t)ENCODER_COUNTS_REV;
    if (d < -(int32_t)(ENCODER_COUNTS_REV / 2)) d += (int32_t)ENCODER_COUNTS_REV;
    return d;
}

/* --- Периферия --- */

static TIM_HandleTypeDef  htim_poll;
static IWDG_HandleTypeDef hiwdg;

/* --- DWT-задержки (блокирующие, для HAL) --- */

#define DWT_MS_CYCLES(ms) ((uint32_t)(ms) * (SYSCLK_HZ / 1000U))

void Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void Delay_ms(uint32_t ms)
{
    while (ms > 0) {
        uint32_t chunk = (ms > 89000U) ? 89000U : ms;
        uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < DWT_MS_CYCLES(chunk)) ;
        ms -= chunk;
    }
}

/**
 * @brief Состояния неблокирующей инициализации.
 */
typedef enum {
    INIT_UART,          ///< Инициализация командного порта USART1
    INIT_BISS_INIT,     ///< Настройка интерфейса энкодера
    INIT_BISS_WAIT,     ///< Ожидание готовности энкодера после подачи питания
    INIT_ENC_DIAG,      ///< Диагностика энкодера перед разрешением движения
    INIT_MOTOR_DRIVER,  ///< Инициализация фасада управления мотором (TMC2209)
    INIT_POLL_TIMER,    ///< Запуск системного таймера опроса (1 кГц)
    INIT_IWDG,          ///< Запуск сторожевого таймера
    INIT_DONE           ///< Инициализация завершена
} InitState;

static InitState s_init = INIT_UART;
static uint32_t  s_init_t0;

/* Ограничение попыток инициализации UART: при неисправном порте автомат
 * иначе навсегда завис бы в INIT_UART (без watchdog и индикации плата
 * выглядит мёртвой). После лимита деградируем — работаем без UART. */
#define UART_INIT_MAX_TRIES 3U
static uint8_t s_uart_tries = 0;
static uint8_t g_uart_fault = 0;   /* 1 = UART не поднялся: быстрое мигание LED */
static uint8_t g_enabled;          /* предварительное объявление: определение с
                                      инициализатором ниже (Init_Poll выше него) */

/**
 * @brief Диагностика энкодера: серия чтений BiSS-C (см. board.h ENCODER_DIAG_*).
 *
 * Проверка «здоровья» энкодера: достаточное число валидных ответов и малый
 * разброс позиции при неподвижном вале. Используется дважды:
 * - при старте (до включения мотора): при провале мотор остаётся выключенным,
 *   хост получает err:enc, хоуминг запрещён;
 * - по команде diag: свежая серия собирается на фоне штатного 1 кГц опроса.
 */
typedef struct {
    uint8_t      active;      ///< 1 = сбор образцов ещё идёт
    uint8_t      passed;      ///< 1 = энкодер прошёл проверку
    uint8_t      samples;     ///< Выполнено чтений
    uint8_t      ok_cnt;      ///< Из них валидных (BISS_OK / BISS_ERR_WARNING)
    BiSS_Status  last_st;     ///< Статус последнего чтения
    float        spread_deg;  ///< Разброс позиции по валидным чтениям, град
    BiSS_Reading last_good;   ///< Последнее валидное чтение (якорь позиции)
    /* Накопители разброса (в отсчётах, относительно первого валидного чтения) */
    int32_t      ref, min, max;
    uint8_t      have_ref;
} EncDiag;

static EncDiag g_enc_diag = { .passed = 0, .last_st = BISS_ERR_SPI }; ///< Стартовая
static EncDiag g_rediag;                                              ///< По команде diag

static void EncDiag_Begin(EncDiag *d)
{
    *d = (EncDiag){ .active = 1, .last_st = BISS_ERR_SPI };
}

/**
 * @brief Учёт одного чтения в серии диагностики.
 * @param[in] st Статус чтения.
 * @param[in] rd Данные чтения (используются только при валидном статусе).
 *
 * После ENCODER_DIAG_SAMPLES чтений вычисляет разброс и вердикт (поле passed),
 * сбрасывая active.
 */
static void EncDiag_Sample(EncDiag *d, BiSS_Status st, const BiSS_Reading *rd)
{
    d->samples++;
    d->last_st = st;

    if (st == BISS_OK || st == BISS_ERR_WARNING) {
        d->ok_cnt++;
        d->last_good = *rd;
        /* Разброс позиции с учётом перехода через ноль счётчика энкодера */
        if (!d->have_ref) {
            d->have_ref = 1;
            d->ref = (int32_t)rd->position;
            d->min = d->max = 0;
        } else {
            int32_t delta = enc_wrap_delta(rd->position, (uint32_t)d->ref);
            if (delta < d->min) d->min = delta;
            if (delta > d->max) d->max = delta;
        }
    }

    if (d->samples < ENCODER_DIAG_SAMPLES)
        return;

    d->active     = 0;
    d->spread_deg = COUNTS_TO_DEG(d->max - d->min);
    d->passed     = (d->ok_cnt >= ENCODER_DIAG_MIN_OK) &&
                    (d->spread_deg <= ENCODER_DIAG_MAX_SPREAD_DEG);
}

static void EncDiag_Report(const EncDiag *d)
{
    if (d->passed)
        SendResponse("enc:ok n=%u/%u spread=%.3f pos=%.2f\r\n",
                     (unsigned)d->ok_cnt, (unsigned)d->samples,
                     (double)d->spread_deg, (double)d->last_good.angle_deg);
    else
        SendResponse("err:enc n=%u/%u spread=%.3f last=%u\r\n",
                     (unsigned)d->ok_cnt, (unsigned)d->samples,
                     (double)d->spread_deg, (unsigned)d->last_st);
}

/**
 * @brief Подкормка повторной диагностики (команда diag) чтениями штатного опроса.
 *
 * Образцы собираются из того же 1 кГц цикла, движение не прерывается
 * (запуск diag разрешён только при остановленном моторе). По завершении
 * серии отчёт уходит хосту той же строкой enc:ok / err:enc, что и при старте.
 */
static void Rediag_Feed(BiSS_Status st, const BiSS_Reading *rd)
{
    if (!g_rediag.active)
        return;
    EncDiag_Sample(&g_rediag, st, rd);
    if (!g_rediag.active)
        EncDiag_Report(&g_rediag);
}

/**
 * @brief Boot-баннер: причина последнего сброса и версия прошивки.
 *
 * Отправляется одной строкой `boot:rst=<флаги> fw=<версия>` после подъёма
 * UART. Делает молчаливые перезагрузки (в т.ч. по IWDG) различимыми на хосте:
 * частый rst=iwdg = зависание, пойманное сторожевым таймером.
 */
static void Boot_Banner(void)
{
    uint32_t csr = RCC->CSR;
    char flags[48];
    int n = 0;
    flags[0] = '\0';
    /* Порядок флагов зафиксирован (por,pin,sft,iwdg,wwdg,lpwr) */
    #define RST_APPEND(bit, name) \
        do { if (csr & (bit)) \
            n += snprintf(flags + n, sizeof(flags) - (size_t)n, "%s" name, n ? "," : ""); \
        } while (0)
    RST_APPEND(RCC_CSR_PORRSTF,  "por");
    RST_APPEND(RCC_CSR_PINRSTF,  "pin");
    RST_APPEND(RCC_CSR_SFTRSTF,  "sft");
    RST_APPEND(RCC_CSR_IWDGRSTF, "iwdg");
    RST_APPEND(RCC_CSR_WWDGRSTF, "wwdg");
    RST_APPEND(RCC_CSR_LPWRRSTF, "lpwr");
    #undef RST_APPEND
    if (flags[0] == '\0') { flags[0] = '-'; flags[1] = '\0'; }
    __HAL_RCC_CLEAR_RESET_FLAGS();
    SendResponse("boot:rst=%s fw=%s\r\n", flags, FW_VERSION);
}

/**
 * @brief Опрос стейт-машины инициализации.
 * @return 1, если инициализация завершена, иначе 0.
 *
 * Функция вызывается в цикле до тех пор, пока не будут настроены все модули.
 * Использует DWT-таймер для неблокирующих задержек.
 */
static uint8_t Init_Poll(void)
{
    switch (s_init) {

    case INIT_UART:
        if (UART_Init() == 0) {
            Boot_Banner();                /* причина сброса + версия FW */
            s_init = INIT_BISS_INIT;
        } else if (++s_uart_tries >= UART_INIT_MAX_TRIES) {
            /* Работаем без UART: слежение за позицией и IWDG живут,
             * мотор не включаем. Индикация — быстрое мигание LED. */
            g_uart_fault = 1;
            g_enabled = 0;
            s_init = INIT_BISS_INIT;
        }
        break;

    case INIT_BISS_INIT: {
        BiSS_Config cfg = {
            .spi_instance    = SPI1,
            .resolution_bits = ENCODER_RESOLUTION_BITS,
            .de_port = XCVR_DE_PORT, .de_pin = XCVR_DE_PIN,
            .re_port = XCVR_RE_PORT, .re_pin = XCVR_RE_PIN,
        };
        BiSS_Init(&cfg);
        s_init_t0 = DWT->CYCCNT;
        s_init = INIT_BISS_WAIT;
        break;
    }

    case INIT_BISS_WAIT:
        if ((DWT->CYCCNT - s_init_t0) >= DWT_MS_CYCLES(ENCODER_STARTUP_MS)) {
            EncDiag_Begin(&g_enc_diag);
            s_init_t0 = DWT->CYCCNT;
            s_init = INIT_ENC_DIAG;
        }
        break;

    case INIT_ENC_DIAG: {
        /* Серия чтений с паузами: проверяем стабильность энкодера ДО включения мотора */
        if ((DWT->CYCCNT - s_init_t0) < DWT_MS_CYCLES(ENCODER_DIAG_INTERVAL_MS))
            break;
        s_init_t0 = DWT->CYCCNT;

        BiSS_Reading rd;
        EncDiag_Sample(&g_enc_diag, BiSS_Read(&rd), &rd);
        if (g_enc_diag.active)
            break;

        EncDiag_Report(&g_enc_diag);
        s_init = INIT_MOTOR_DRIVER;
        break;
    }

    case INIT_MOTOR_DRIVER: {
        int r = tmc2209_motor_init();
        if (r != 0) {
            SendResponse("err:motor_init=%d\r\n", r);
        }
        s_init = INIT_POLL_TIMER;
        break;
    }

    case INIT_POLL_TIMER:
        PollTimer_Init();
        /* Чистим флаг обновления от Base_Init (EGR UG), иначе ISR сработает сразу */
        __HAL_TIM_CLEAR_FLAG(&htim_poll, TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start_IT(&htim_poll);
        s_init = INIT_IWDG;
        break;

    case INIT_IWDG:
        hiwdg.Instance       = IWDG;
        hiwdg.Init.Prescaler = IWDG_PRESCALER;
        hiwdg.Init.Reload    = IWDG_RELOAD;
        HAL_IWDG_Init(&hiwdg);
        s_init = INIT_DONE;
        break;

    case INIT_DONE:
        return 1;
    }
    return 0;
}

/**
 * @brief Глобальное состояние системы.
 */
typedef enum { 
    MODE_CL = 0,    ///< Closed-Loop: PID-регулирование по энкодеру
    MODE_OL = 1     ///< Open-Loop: прямое управление без ОС (счётчик шагов)
} CtrlMode;

/* Состояние PID-регулятора */
static PID_State g_pid = {
    .kp = PID_KP_DEFAULT, .ki = PID_KI_DEFAULT, .kd = PID_KD_DEFAULT,
    .integral = 0, .prev_error = 0,
    .output_min = -MAX_DEG_PER_TICK, .output_max = MAX_DEG_PER_TICK,
    .initialized = 0
};

static double   g_target_deg     = 0;           ///< Целевая позиция (градусы), double — накопитель
static uint8_t  g_enabled        = 1;           ///< Флаг разрешения работы (движение разрешено)
static uint16_t g_output_period  = OUTPUT_PERIOD_MS_DEFAULT; ///< Период телеметрии
static uint8_t  g_telem_debug    = TELEMETRY_DEBUG_DEFAULT;  ///< Флаг расширенной телеметрии

static uint32_t g_enc_raw_prev   = 0xFFFFFFFF;  ///< Предыдущее "сырое" значение энкодера (для дельты)
static int64_t  g_enc_counts     = 0;           ///< Накопленный счетчик импульсов (многооборотность)

static float    g_last_ctrl      = 0;           ///< Последнее вычисленное управляющее воздействие

static CtrlMode g_mode           = MODE_CL;     ///< Текущий режим управления
static uint32_t g_enc_fail_cnt   = 0;           ///< Счетчик пропусков данных энкодера
static uint8_t  g_enc_outlier    = 0;           ///< Последнее чтение отброшено фильтром выбросов
static uint32_t g_outlier_cnt    = 0;           ///< Всего отфильтровано выбросов (телеметрия of)
static uint32_t g_sample_t       = 0;           ///< DWT-момент сэмпла обрабатываемого чтения (фильтр выбросов)
static double   g_ol_pos         = 0;           ///< Виртуальная позиция для Open-Loop (double — накопитель)
static uint8_t  g_was_outside_db = 0;           ///< Флаг выхода из мертвой зоны (для логики доводки)
static uint8_t  g_homing         = 0;           ///< Флаг процесса выхода в "дом" после инициализации

static int8_t   g_cont_dir       = 0;           ///< Направление непрерывного вращения (t+/t-)

/* --- Профиль движения (команды v= и a=) --- */

static float    g_vmax_deg_s     = SPEED_DEFAULT_DEG_S; ///< Предел скорости, град/с
static float    g_accel_deg_s2   = ACCEL_DEFAULT_DEG_S2;///< Предел ускорения, град/с² (0 = выкл)
static float    g_vel_cmd        = 0;           ///< Текущая слew-скорость, град/тик

static inline float vmax_per_tick(void)
{ return g_vmax_deg_s * DT_S; }

/**
 * @brief Ограничитель скорости и ускорения (вызывается каждый тик движения).
 * @param[in] v_des    Желаемая скорость, град/тик (выход PID / ошибка OL / джог).
 * @param[in] err      Ошибка позиции до цели, град (для тормозного конверта).
 * @param[in] have_err 0 = цели нет (непрерывное вращение), конверт не применять.
 * @return Разрешённая скорость на этот тик, град/тик.
 *
 * Скорость всегда ограничивается ±g_vmax_deg_s. При g_accel_deg_s2 > 0
 * дополнительно: изменение скорости за тик не быстрее a·dt (рампа) и, если
 * есть цель, |v| ≤ sqrt(2·a·|err|) — чтобы успеть затормозить без перелёта.
 */
static float Motion_Limit(float v_des, float err, uint8_t have_err)
{
    float vmax = vmax_per_tick();
    v_des = clampf(v_des, -vmax, vmax);

    if (g_accel_deg_s2 > 0) {
        if (have_err) {
            float ae = g_accel_deg_s2 * (err < 0 ? -err : err);
            float vbrake = sqrtf(2.0f * ae) * DT_S;
            v_des = clampf(v_des, -vbrake, vbrake);
        }
        float dv = g_accel_deg_s2 * DT_S * DT_S; /* приращение за тик, град/тик */
        g_vel_cmd += clampf(v_des - g_vel_cmd, -dv, dv);
    } else {
        g_vel_cmd = v_des;
    }
    return g_vel_cmd;
}

/**
 * @brief Состояние логики автоматического сканирования.
 */
typedef enum { 
    SCAN_IDLE,      ///< Сканирование не запущено
    SCAN_MOVING,    ///< Ожидание достижения мотором целевой позиции
    SCAN_DELAY      ///< Ожидание в целевой позиции (задержка перед следующим шагом)
} ScanState;

static ScanState g_scan_st       = SCAN_IDLE;
static double    g_scan_cur      = 0;           ///< Текущая целевая точка сканирования (double — накопитель)
static float     g_scan_start    = 0;           ///< Начало сектора
static float     g_scan_end      = 0;           ///< Конец сектора
static float     g_scan_step     = 0;           ///< Шаг сканирования
static uint16_t  g_scan_delay_ms = 0;           ///< Время ожидания в точке
static uint16_t  g_scan_delay_cnt= 0;           ///< Счетчик времени ожидания
static int8_t    g_scan_dir      = 1;           ///< Текущее направление (1/-1) для зигзага
static int8_t    g_scan_inf      = 0;           ///< Флаг бесконечного сканирования (направление)

static void ApplyVelocity(float v_deg_per_tick)
{
    if (v_deg_per_tick == 0.0f) {
        tmc2209_motor_stop();
        g_last_ctrl = 0.0f;
        return;
    }
    float av = v_deg_per_tick;
    if (av < 0.0f) av = -av;
    uint32_t sps = (uint32_t)(av / g_deg_per_step * (float)POLL_FREQ_HZ + 0.5f);
    if (sps == 0U) {
        tmc2209_motor_stop();
        g_last_ctrl = 0.0f;
        return;
    }
    tmc2209_motor_set_step_rate(sps, v_deg_per_tick > 0.0f ? 1 : 0);
    g_last_ctrl = v_deg_per_tick;
}

/** Одноразовая серия шагов (доводка в deadband). */
static void DoSteps(int32_t steps)
{
    if (steps == 0) {
        tmc2209_motor_stop();
        g_last_ctrl = 0.0f;
        return;
    }
    tmc2209_motor_move_steps(steps);
    g_last_ctrl = (float)steps * g_deg_per_step;
}

/* Сброс регулятора и профиля движения (смена цели/режима, остановки) */
static void Control_Reset(void)
{
    PID_Reset(&g_pid);
    g_vel_cmd  = 0;
}

/* --- Сэмплирование энкодера в ISR TIM2 (жёсткий real-time) --- */

/* Конечный автомат обмена с энкодером ведётся в прерывании TIM2 (1 кГц):
 * забрать готовый результат DMA-чтения BiSS и тут же стартовать следующее.
 * Это держит детерминированный интервал сэмпла независимо от загрузки
 * главного цикла (в т.ч. во время блокирующего UART-обмена с TMC2209).
 * Тяжёлая обработка (PID/мотор/стол/скан/телеметрия) — в главном цикле по
 * флагу s_tick_ready. */
static uint8_t          s_biss_pending = 0;   ///< В полёте DMA-чтение BiSS
static uint8_t          s_biss_ticks   = 0;   ///< Тиков ожидания текущего чтения (таймаут)
static uint32_t         s_read_start_t = 0;   ///< DWT-момент старта текущего чтения

/* Снимок последнего завершённого чтения: ISR пишет, main читает под коротким
 * запретом прерываний. */
static volatile uint8_t s_tick_ready  = 0;    ///< ISR → main: пора обработать тик
static BiSS_Reading     s_sample_rd;          ///< Данные последнего завершённого чтения
static BiSS_Status      s_sample_st   = BISS_ERR_SPI;
static uint32_t         s_sample_t    = 0;    ///< DWT-момент сэмпла s_sample_rd
static volatile uint8_t s_sample_new  = 0;    ///< Есть свежий результат для main
static volatile uint8_t s_sample_fail = 0;    ///< Старт чтения не удался (для diag)

/**
 * @brief Обработчик тика TIM2 (1 кГц) — сэмплирование энкодера.
 *
 * Забирает готовый результат DMA-чтения BiSS, немедленно стартует следующее
 * (детерминированный интервал сэмпла) и поднимает флаг s_tick_ready. Зависшее
 * чтение (сбой SPI/DMA) прерывает по таймауту, иначе следующее не стартует и
 * сэмплирование встало бы навсегда.
 */
static void Sample_ISR(void)
{
    if (s_biss_pending && !BiSS_IsReady() &&
        ++s_biss_ticks >= BISS_DMA_TIMEOUT_TICKS)
        BiSS_Abort();   /* выставит done+error — результат заберём ниже */

    if (s_biss_pending && BiSS_IsReady()) {
        s_sample_st    = BiSS_GetResult(&s_sample_rd);
        s_sample_t     = s_read_start_t;   /* момент сэмпла завершённого чтения */
        s_sample_new   = 1;
        s_biss_pending = 0;
    }

    if (!s_biss_pending) {
        s_read_start_t = DWT->CYCCNT;      /* момент сэмпла нового чтения */
        s_biss_pending = (BiSS_StartRead() == 0);
        s_biss_ticks   = 0;
        if (!s_biss_pending)
            s_sample_fail = 1;             /* SPI не стартовал — тоже результат для diag */
    }

    s_tick_ready = 1;
}

/* --- MAIN --- */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    BSP_Init();
    Delay_Init();

    BiSS_Status  st        = BISS_ERR_SPI;
    uint8_t      enc_ok    = 0;
    uint8_t      init_done = 0;

    while (1) {
        if (!init_done) {
            if (Init_Poll()) {
                if (g_enc_diag.passed) {
                    /* Якорь позиции — последнее валидное чтение диагностики */
                    st = g_enc_diag.last_good.status;
                    Encoder_Accumulate(&g_enc_diag.last_good);
                } else {
                    /* Диагностика не пройдена: мотор не включаем, хоуминг
                     * запрещён (см. Encoder_Accumulate) — ждём оператора. */
                    st = g_enc_diag.last_st;
                    g_enabled = 0;
                }
                /* Первое чтение стартует уже ISR TIM2 (включён в INIT_POLL_TIMER) */
                enc_ok    = (g_enc_raw_prev != 0xFFFFFFFF);
                init_done = 1;
                if (g_enabled) tmc2209_motor_set_enabled(1);
            }
            UART_Task();
            continue;
        }

        UART_Task();
        /* Диагностика TMC2209 — блокирующий UART-обмен: разрешаем его только
         * в простое (не во время сканирования/вращения). Непрерывный STEP-
         * генератор при этом продолжает крутиться на текущей частоте, а
         * сэмплирование энкодера идёт в ISR TIM2 — блокировка цикла не рвёт
         * ни поток импульсов, ни интервал сэмпла. */
        if (g_scan_st == SCAN_IDLE && g_cont_dir == 0)
            tmc2209_motor_task();
        PollCommands();

        if (!s_tick_ready)
            continue;

        /* Снимок результата сэмплирования из ISR (короткая критическая секция) */
        BiSS_Reading rd;
        BiSS_Status  st_new;
        uint8_t      have_new, start_fail;
        __disable_irq();
        rd           = s_sample_rd;
        st_new       = s_sample_st;
        g_sample_t   = s_sample_t;
        have_new     = s_sample_new;   s_sample_new  = 0;
        start_fail   = s_sample_fail;  s_sample_fail = 0;
        s_tick_ready = 0;
        __enable_irq();

        if (have_new) {
            st = st_new;
            enc_ok = (st == BISS_OK || st == BISS_ERR_WARNING);
            g_enc_outlier = 0;
            if (enc_ok && !Encoder_FilterOutlier(&rd)) {
                /* Показание физически невозможно — отброшено как выброс */
                enc_ok        = 0;
                g_enc_outlier = 1;
            }
            if (enc_ok) Encoder_Accumulate(&rd);
            Rediag_Feed(st, &rd);
        } else {
            /* Свежих данных в этом тике нет (обмен ещё в полёте) */
            enc_ok = 0;
        }
        if (start_fail) {
            /* SPI не стартовал — для diag это тоже результат (ошибка) */
            BiSS_Reading dummy = {0};
            Rediag_Feed(BISS_ERR_SPI, &dummy);
        }

        Mode_Update(enc_ok);
        MotorControl_Tick(enc_ok);
        Stall_Tick(enc_ok);
        Scan_Tick();
        Telemetry_Tick(enc_ok, st);
        Heartbeat_Tick();
        HAL_IWDG_Refresh(&hiwdg);
    }
}

/* --- Многооборотная позиция --- */

/**
 * @brief Обработка данных энкодера и расчет многооборотной позиции.
 * @param[in] rd Текущее чтение с энкодера (уже прошедшее фильтр выбросов).
 *
 * Вычисляет разность между текущим и предыдущим значением с учетом переполнения
 * (rollover) 17-битного счетчика. При первом вызове производит привязку позиции
 * и расчет целевой точки для плавного старта.
 */
static void Encoder_Accumulate(const BiSS_Reading *rd)
{
    if (g_enc_raw_prev == 0xFFFFFFFF) {
        g_enc_counts   = (int64_t)rd->position;
        g_enc_raw_prev = rd->position;
        double pos = COUNTS_TO_DEG_D(g_enc_counts);
        g_ol_pos = pos;
        if (g_enc_diag.passed) {
            /* Автоматический выход в «дом» разрешён только после успешной
             * стартовой диагностики энкодера. */
            g_target_deg = pos + shortest_path_err(STARTUP_TARGET_OFFSET_DEG, (float)pos);
            g_homing     = 1;
        } else {
            /* Энкодер восстановился после провала диагностики: привязываемся
             * на месте, движение — только по явной команде оператора. */
            g_target_deg = pos;
            g_homing     = 0;
        }
        return;
    }

    g_enc_counts  += enc_wrap_delta(rd->position, g_enc_raw_prev);
    g_enc_raw_prev = rd->position;
}

/* --- Фильтр выбросов показаний энкодера --- */

/* Пороги в отсчётах энкодера (позиция за 1 мс и потолок неоднозначности wrap) */
#define OUTLIER_DELTA_COUNTS ((int32_t)(ENCODER_OUTLIER_MAX_DELTA_DEG / 360.0f * \
                                        (float)ENCODER_COUNTS_REV))
#define OUTLIER_BUDGET_MAX_MS 20U   /* 20 мс × 3° = 60°, дальше wrap неоднозначен */
#define OUTLIER_MS_CYCLES     (SYSCLK_HZ / 1000U)

static uint32_t s_out_t0      = 0;  ///< Момент сэмпла последнего принятого чтения (DWT)
static uint32_t s_pend_raw    = 0;  ///< Кандидат новой позиции (raw)
static uint8_t  s_pend_streak = 0;  ///< Согласных чтений кандидата подряд
static uint32_t s_pend_t0     = 0;  ///< Момент сэмпла последнего чтения кандидата (DWT)

/* Бюджет сдвига по фактическому времени между сэмплами: elapsed_ms × MAX_DELTA.
 * Время (DWT), а не число исполненных тиков: пауза главного цикла (например,
 * UART-обмен с TMC2209) растягивает интервал между сэмплами, и валидное чтение
 * при быстром движении иначе выглядело бы выбросом. */
static int64_t outlier_budget(uint32_t since_cycles)
{
    uint32_t ms = since_cycles / OUTLIER_MS_CYCLES;
    if (ms < 1U) ms = 1U;
    if (ms > OUTLIER_BUDGET_MAX_MS) ms = OUTLIER_BUDGET_MAX_MS;
    return (int64_t)ms * OUTLIER_DELTA_COUNTS;
}

/**
 * @brief Фильтр выбросов: правдоподобно ли новое показание энкодера?
 * @param[in] rd Чтение со статусом BISS_OK / BISS_ERR_WARNING (сэмпл g_sample_t).
 * @return 1 — принять (передать в Encoder_Accumulate), 0 — отбросить.
 *
 * Показание отбрасывается, если сдвиг от последней принятой позиции превышает
 * физически возможный (ENCODER_OUTLIER_MAX_DELTA_DEG за 1 мс, бюджет
 * масштабируется на реальное время между сэмплами, но не более 60° — дальше
 * дельта неоднозначна из-за перехода через ноль). Единичные выбросы (CRC6
 * слабая — искажённый кадр проходит с вероятностью ~1/64) не дёргают контур.
 *
 * Реальный скачок позиции (проскальзывание муфты, восстановление энкодера
 * после долгой потери связи на ходу) отличается от выброса устойчивостью:
 * ENCODER_OUTLIER_STREAK согласных чтений подряд — и позиция перепривязывается
 * штатным путём (дельта через Encoder_Accumulate).
 */
static uint8_t Encoder_FilterOutlier(const BiSS_Reading *rd)
{
    /* Первое чтение — якорь, фильтровать не с чем */
    if (g_enc_raw_prev == 0xFFFFFFFF) {
        s_out_t0      = g_sample_t;
        s_pend_streak = 0;
        return 1;
    }

    int32_t d = enc_wrap_delta(rd->position, g_enc_raw_prev);
    if (d < 0) d = -d;
    if (d <= outlier_budget(g_sample_t - s_out_t0)) {
        s_out_t0      = g_sample_t;
        s_pend_streak = 0;
        return 1;
    }

    /* Вне бюджета: копим подтверждения новой позиции */
    if (s_pend_streak) {
        int32_t pd = enc_wrap_delta(rd->position, s_pend_raw);
        if (pd < 0) pd = -pd;
        s_pend_streak = (pd <= outlier_budget(g_sample_t - s_pend_t0))
                        ? (uint8_t)(s_pend_streak + 1U) : 1U;
    } else {
        s_pend_streak = 1;
    }
    s_pend_raw = rd->position;
    s_pend_t0  = g_sample_t;

    if (s_pend_streak >= ENCODER_OUTLIER_STREAK) {
        /* Новая позиция устойчива — это реальное движение, принимаем */
        s_out_t0      = g_sample_t;
        s_pend_streak = 0;
        return 1;
    }

    g_outlier_cnt++;
    return 0;
}

/* --- CL ↔ OL --- */

/**
 * @brief Управление переключением режимов CL/OL.
 * @param[in] enc_ok Флаг валидности данных энкодера в текущем тике.
 *
 * Если энкодер не отвечает более ENCODER_FAIL_MS, система переходит в Open-Loop.
 * При восстановлении связи возвращается в Closed-Loop со сбросом PID.
 */
static void Mode_Update(uint8_t enc_ok)
{
    if (enc_ok) {
        g_enc_fail_cnt = 0;
        if (g_mode == MODE_OL) {
            g_mode = MODE_CL;
            Control_Reset();
        }
    } else {
        g_enc_fail_cnt++;
        if (g_mode == MODE_CL && g_enc_fail_cnt >= (ENCODER_FAIL_MS / POLL_INTERVAL_MS)) {
            g_mode = MODE_OL;
            g_ol_pos = COUNTS_TO_DEG_D(g_enc_counts);
        }
    }
}

/* --- Управление мотором --- */

/**
 * @brief Основной расчет управляющего воздействия мотора (1 кГц).
 * @param[in] enc_ok Флаг валидности данных энкодера.
 *
 * Реализует:
 * 1. Остановку, если g_enabled == 0.
 * 2. Непрерывное вращение (режим t+/t-), если g_cont_dir != 0.
 * 3. Closed-Loop по PID-алгоритму, если есть данные энкодера.
 * 4. Open-Loop (программный счетчик шагов), если данных энкодера нет.
 *
 * Включает логику "мертвой зоны" (deadband) и доводки до целевой позиции.
 */
static void MotorControl_Tick(uint8_t enc_ok)
{
    if (!g_enabled) {
        tmc2209_motor_stop();
        g_last_ctrl = 0;
        return;
    }
    g_last_ctrl = 0;

    if (g_cont_dir != 0) {
        float v = Motion_Limit((float)g_cont_dir * vmax_per_tick(), 0, 0);
        ApplyVelocity(v);
        g_ol_pos    += v;
        g_target_deg+= v;
        if (g_ol_pos > 1e7f || g_ol_pos < -1e7f) {
            g_ol_pos = g_target_deg = 0;
            g_enc_counts = (int64_t)g_enc_raw_prev;
        }
        return;
    }

    if (g_mode == MODE_CL && enc_ok) {
        /* Вычитание в double, результат (малая ошибка) — во float */
        float err = (float)(g_target_deg - COUNTS_TO_DEG_D(g_enc_counts));

        if (err > PID_DEADBAND_DEG || err < -PID_DEADBAND_DEG) {
            g_was_outside_db = 1;
            float pid = PID_Update(&g_pid, err, DT_S);
            float v = Motion_Limit(pid, err, 1);
            ApplyVelocity(v);
            if (g_scan_st == SCAN_MOVING && v != 0.0f)
                HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
        } else {
            uint8_t final_move = 0;
            if (g_was_outside_db) {
                if (err > 0.02f || err < -0.02f) {
                    int32_t snap = DegToSteps(err);
                    if (snap == 0)
                        snap = (err > 0) ? 1 : -1;
                    if (g_scan_st == SCAN_MOVING)
                        HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
                    DoSteps(snap);
                    g_last_ctrl = (float)snap * g_deg_per_step;
                    final_move  = 1;
                }
                g_was_outside_db = 0;
            }
            if (g_homing) {
                g_homing = 0;
                g_enc_counts = (int64_t)g_enc_raw_prev;
                g_target_deg = STARTUP_TARGET_OFFSET_DEG;
                float p = COUNTS_TO_DEG(g_enc_counts);
                if (p - g_target_deg >  180.0f) g_enc_counts -= (int64_t)ENCODER_COUNTS_REV;
                if (p - g_target_deg < -180.0f) g_enc_counts += (int64_t)ENCODER_COUNTS_REV;
            }
            if (g_scan_st == SCAN_MOVING) {
                HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_SET);
                g_scan_cur       = g_target_deg;
                g_scan_st        = SCAN_DELAY;
                g_scan_delay_cnt = 0;
            }
            if (!final_move)
                tmc2209_motor_stop();
            Control_Reset();
        }

    } else if (g_mode == MODE_CL) {
        /* CL без свежих данных энкодера: PWM STEP продолжает идти на последней
         * частоте (коастинг сглаживает единичные пропуски чтения), но не дольше
         * ENCODER_COAST_MS — далее стоп до восстановления данных либо до
         * перехода в OL по ENCODER_FAIL_MS. Иначе мотор до 500 мс крутился бы
         * неуправляемо по устаревшей уставке скорости. */
        if (g_enc_fail_cnt >= ENCODER_COAST_TICKS) {
            /* Только стоп мотора; движение продолжится после восстановления
             * данных энкодера (цель и профиль сохраняются). */
            tmc2209_motor_stop();
        }

    } else if (g_mode == MODE_OL) {
        float err = (float)(g_target_deg - g_ol_pos);

        if (err > PID_DEADBAND_DEG || err < -PID_DEADBAND_DEG) {
            float v = Motion_Limit(err, err, 1);
            ApplyVelocity(v);
            if (g_scan_st == SCAN_MOVING && v != 0.0f)
                HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
            g_ol_pos += v;
        } else {
            if (g_homing) {
                g_homing = 0;
                float p = COUNTS_TO_DEG(g_enc_raw_prev);
                if (p - STARTUP_TARGET_OFFSET_DEG >  180.0f) p -= 360.0f;
                if (p - STARTUP_TARGET_OFFSET_DEG < -180.0f) p += 360.0f;
                g_ol_pos = p;
            }
            if (g_scan_st == SCAN_MOVING) {
                HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_SET);
                g_scan_cur       = g_target_deg;
                g_scan_st        = SCAN_DELAY;
                g_scan_delay_cnt = 0;
            }
            tmc2209_motor_stop();
        }
    }
}

/* --- Защита от блокировки вала (stall) --- */

static float    s_stall_cmd_deg  = 0;  ///< Скомандованный путь за окно, град
static float    s_stall_peak_deg = 0;  ///< Пик смещения по энкодеру от начала окна, град
static int64_t  s_stall_ref;           ///< Позиция энкодера (counts) в начале окна
static uint8_t  s_stall_have_ref = 0;
static uint32_t s_stall_ticks    = 0;  ///< Длительность текущего окна, тиков
static uint16_t s_stall_idle     = 0;  ///< Тиков подряд без команды движения
static char     s_stall_msg[64];       ///< Отчёт об ошибке, ждущий места в UART TX
/* Защёлка stall для телеметрии: удерживает ec = ERR_STALL после срабатывания
 * защиты (драйвер выключен) до явной команды en. Иначе ec на следующем же
 * тике вернулся бы к ERR_OK (энкодер-то отвечает), и причина остановки в
 * поле ec пропала бы. */
static uint8_t  g_stall_latched  = 0;

static void Stall_Reset(void)
{
    s_stall_cmd_deg  = 0;
    s_stall_peak_deg = 0;
    s_stall_have_ref = 0;
    s_stall_ticks    = 0;
    s_stall_idle     = 0;
}

/**
 * @brief Детектор заблокированного вала (вызывается каждый тик, 1 кГц).
 * @param[in] enc_ok Флаг валидности данных энкодера в текущем тике.
 *
 * Принцип: в окне накапливается скомандованный путь (|g_last_ctrl| за тик)
 * и пиковое фактическое смещение по энкодеру относительно начала окна.
 * Пока фактическое движение поспевает за командой, окно сбрасывается
 * (мотор здоров). Вердикт «заблокирован» — по одному из двух критериев:
 *  - быстрый: скомандовано ≥ STALL_FAST_CMD_DEG, а вал сдвинулся меньше
 *    STALL_STUCK_DEG за ≥ STALL_FAST_MS — полная блокировка, реакция за ~200 мс;
 *  - медленный: за полное окно STALL_TIMEOUT_MS скомандовано ≥ STALL_MIN_CMD_DEG,
 *    а факт < STALL_MEAS_FRACTION команды — вал ползёт, но не поспевает.
 * При срабатывании драйвер выключается (как по команде dis), хосту уходит
 * err:stall. Возврат — командой en.
 *
 * Пиковое смещение (а не сумма подвижек за тики) выбрано, чтобы шум
 * энкодера на стоящем вале не накапливался в «фиктивное движение».
 * В open-loop защита не действует (нет данных о фактическом положении).
 */
static void Stall_Tick(uint8_t enc_ok)
{
    /* Отчёт мог не пролезть в переполненную очередь TX — досылаем */
    if (s_stall_msg[0] != '\0' &&
        UART_Transmit((const uint8_t *)s_stall_msg, (uint16_t)strlen(s_stall_msg)) == 0)
        s_stall_msg[0] = '\0';

    if (!g_enabled || g_mode != MODE_CL) {
        Stall_Reset();
        return;
    }

    /* Фактическое смещение вала от начала окна (по валидным чтениям) */
    if (enc_ok) {
        if (!s_stall_have_ref) {
            s_stall_have_ref = 1;
            s_stall_ref      = g_enc_counts;
        } else {
            float net = COUNTS_TO_DEG(g_enc_counts - s_stall_ref);
            if (net < 0) net = -net;
            if (net > s_stall_peak_deg) s_stall_peak_deg = net;
        }
    }

    /* Скомандованное движение в этом тике */
    float cmd = (g_last_ctrl < 0) ? -g_last_ctrl : g_last_ctrl;
    if (cmd > 0) {
        s_stall_cmd_deg += cmd;
        s_stall_idle = 0;
    } else if (++s_stall_idle >= STALL_IDLE_RESET_TICKS) {
        /* Мотор штатно стоит (цель достигнута, пауза скана) — окно неактуально */
        Stall_Reset();
        return;
    }
    s_stall_ticks++;

    /* Быстрый путь: вал практически неподвижен при заметной скомандованной
     * команде — полная блокировка, реагируем за STALL_FAST_MS, не дожидаясь
     * полного окна STALL_TIMEOUT_MS. */
    uint8_t blocked = (s_stall_cmd_deg  >= STALL_FAST_CMD_DEG) &&
                      (s_stall_peak_deg <  STALL_STUCK_DEG)    &&
                      (s_stall_ticks    >= STALL_FAST_TICKS);

    if (!blocked) {
        /* Медленный путь: частичный стопор (вал ползёт, но отстаёт) — по доле
         * пройденного пути за полное окно. */
        if (s_stall_cmd_deg < STALL_MIN_CMD_DEG)
            return;

        if (s_stall_peak_deg >= s_stall_cmd_deg * STALL_MEAS_FRACTION) {
            /* Вал следует за командой — начинаем окно заново */
            Stall_Reset();
            return;
        }

        if (s_stall_ticks < STALL_TIMEOUT_TICKS)
            return;
    }

    /* Вал заблокирован: обесточиваем драйвер и докладываем */
    g_enabled     = 0;
    g_stall_latched = 1;           /* держим ec = ERR_STALL до команды en */
    g_cont_dir = 0;
    g_scan_st  = SCAN_IDLE;
    g_scan_inf = 0;
    g_homing   = 0;
    tmc2209_motor_stop();
    tmc2209_motor_set_enabled(0);
    Control_Reset();
    g_target_deg = COUNTS_TO_DEG_D(g_enc_counts);
    HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);

    snprintf(s_stall_msg, sizeof(s_stall_msg),
             "err:stall cmd=%.1f meas=%.2f pos=%.2f\r\n",
             (double)s_stall_cmd_deg, (double)s_stall_peak_deg,
             (double)g_target_deg);
    if (UART_Transmit((const uint8_t *)s_stall_msg, (uint16_t)strlen(s_stall_msg)) == 0)
        s_stall_msg[0] = '\0';

    Stall_Reset();
}

/* --- Телеметрия --- */

static uint32_t g_dropped_tx = 0;

static void TransmitAll(const uint8_t *buf, uint16_t len)
{
    if (UART_Transmit(buf, len) != 0) {
        g_dropped_tx++;
        return;
    }
    UART_Task();
}

static void Telemetry_Tick(uint8_t enc_ok, BiSS_Status st)
{
    static uint16_t cnt = 0;
    if (g_output_period == 0) return;
    /* Даём коротким CLI-ответам уйти целой строкой, не перемешивая их с телеметрией. */
    if (UART_TxPending()) return;
    /* При полной телеметрии (debug=1) используем не меньший период, чтобы длинные строки успевали по UART */
    uint16_t period = g_telem_debug ? (g_output_period >= OUTPUT_PERIOD_MS_DEBUG_MIN ? g_output_period : OUTPUT_PERIOD_MS_DEBUG_MIN) : g_output_period;
    if (++cnt < period) return;
    cnt = 0;

    char buf[160];
    /* Позиция по режиму: в CL — последняя валидная позиция энкодера (не
     * дёргается при единичном пропуске чтения), в OL — виртуальный счётчик. */
    double deg = (g_mode == MODE_CL) ? COUNTS_TO_DEG_D(g_enc_counts) : g_ol_pos;
    /* Приоритет: активная ошибка энкодера важнее защёлки stall (иначе новый
     * обрыв связи с энкодером скрылся бы за «старой» блокировкой вала). */
    uint8_t ec;
    if (!enc_ok)
        ec = g_enc_outlier ? (uint8_t)ERR_ENC_OUTLIER : (uint8_t)st;
    else if (g_stall_latched)
        ec = (uint8_t)ERR_STALL;
    else
        ec = (uint8_t)ERR_OK;
    int len;

    if (g_telem_debug)
        len = snprintf(buf, sizeof(buf),
            "cp:%.2f,tp:%.2f,pe:%.2f,u:%.4f,m:%s,ec:%u,kp:%.4f,ki:%.4f,kd:%.4f,v:%.1f,a:%.1f,of:%lu,drp:%lu\r\n",
            (double)deg, (double)g_target_deg, (double)(g_target_deg - deg),
            (double)g_last_ctrl, (g_mode == MODE_CL) ? "cl" : "ol",
            (unsigned)ec, (double)g_pid.kp, (double)g_pid.ki, (double)g_pid.kd,
            (double)g_vmax_deg_s, (double)g_accel_deg_s2,
            (unsigned long)g_outlier_cnt, (unsigned long)g_dropped_tx);
    else
        len = snprintf(buf, sizeof(buf), "cp:%.2f,ec:%u\r\n", (double)deg, (unsigned)ec);

    if (len >= (int)sizeof(buf)) len = (int)sizeof(buf) - 1;  /* snprintf усёк */
    if (len > 0) TransmitAll((uint8_t *)buf, (uint16_t)len);
}

static void Heartbeat_Tick(void)
{
    static uint32_t cnt = 0;
    /* При отказе UART период мигания в 5 раз меньше — отличимый код неисправности. */
    uint32_t period = g_uart_fault ? (LED_TOGGLE_INTERVAL / 5U) : LED_TOGGLE_INTERVAL;
    if (++cnt >= period) {
        cnt = 0;
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }
}

/* --- Сканирование --- */

static void Scan_Tick(void)
{
    if (g_scan_st != SCAN_DELAY) return;
    /* Задержка в мс: тик 1 кГц, считаем до g_scan_delay_ms включительно */
    if (g_scan_delay_ms == 0) return; /* не должно быть при валидной команде */
    if (++g_scan_delay_cnt < g_scan_delay_ms) return;

    double next;
    if (g_scan_inf != 0) {
        next = g_scan_cur + (double)g_scan_inf * g_scan_step;
        if (next > 1e7 || next < -1e7) {
            double off = g_scan_cur;
            g_scan_cur   -= off;
            next         -= off;
            g_ol_pos     -= off;
            g_target_deg -= off;
            g_enc_counts -= (int64_t)(off * (double)ENCODER_COUNTS_REV / 360.0
                                      + (off >= 0 ? 0.5 : -0.5));
        }
    } else {
        next = g_scan_cur + (float)g_scan_dir * g_scan_step;
        if (g_scan_dir > 0 && next > g_scan_end) {
            next = (g_scan_cur >= g_scan_end) ? (g_scan_end - g_scan_step) : g_scan_end;
            g_scan_dir = -1;
        } else if (g_scan_dir < 0 && next < g_scan_start) {
            next = (g_scan_cur <= g_scan_start) ? (g_scan_start + g_scan_step) : g_scan_start;
            g_scan_dir = 1;
        }
    }

    g_target_deg = next;
    g_scan_cur   = next;
    g_scan_st    = SCAN_MOVING;
    g_scan_delay_cnt = 0;
    HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
    /* Профиль движения не сбрасываем: между точками скана рампа скорости
     * продолжается плавно, PID стартует с чистым интегралом */
    PID_Reset(&g_pid);
}

/* --- Команды --- */

static void PollCommands(void)
{
    char line[64];
    Cmd_Result cmd;
    if (UART_ReadLine(line, sizeof(line)) > 0 && Cmd_Parse(line, &cmd))
        ProcessCommand(&cmd);
}

static void SendResponse(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (len > 0) TransmitAll((uint8_t *)buf, (uint16_t)len);
}

static void ProcessCommand(const Cmd_Result *cmd)
{
    switch (cmd->type) {

    case CMD_ENABLE:
        g_enabled  = 1;
        g_stall_latched = 0;       /* en — штатное восстановление после stall */
        g_cont_dir = 0;
        tmc2209_motor_set_enabled(1);
        Control_Reset();
        g_target_deg = (g_mode == MODE_CL) ? COUNTS_TO_DEG_D(g_enc_counts) : g_ol_pos;
        g_homing     = 0;
        SendResponse("ok:en\r\n");
        break;

    case CMD_DISABLE:
        g_enabled  = 0;
        g_cont_dir = 0;
        tmc2209_motor_stop();
        tmc2209_motor_set_enabled(0);
        SendResponse("ok:dis\r\n");
        break;

    case CMD_SET_TARGET:
        g_cont_dir   = 0;
        g_target_deg = cmd->target;
        g_homing     = 0;
        PID_Reset(&g_pid);
        /* g_vel_cmd не трогаем: смена цели на ходу продолжает рампу плавно */
        SendResponse("ok:t=%.2f\r\n", (double)g_target_deg);
        break;

    case CMD_CONTINUOUS:
        g_cont_dir = cmd->continuous_dir;
        g_scan_st  = SCAN_IDLE;
        g_homing   = 0;
        PID_Reset(&g_pid);
        if (!g_enabled) { g_enabled = 1; tmc2209_motor_set_enabled(1); }
        SendResponse("ok:t=%c\r\n", (g_cont_dir > 0) ? '+' : '-');
        break;

    case CMD_SET_KP:
        g_pid.kp = cmd->kp;
        SendResponse("ok:kp=%.4f\r\n", (double)g_pid.kp);
        break;

    case CMD_SET_KI:
        g_pid.ki = cmd->ki;
        SendResponse("ok:ki=%.4f\r\n", (double)g_pid.ki);
        break;

    case CMD_SET_KD:
        g_pid.kd = cmd->kd;
        SendResponse("ok:kd=%.4f\r\n", (double)g_pid.kd);
        break;

    case CMD_SET_OUTPUT_PERIOD:
        g_output_period = cmd->output_period_ms;
        SendResponse("ok:op=%u\r\n", (unsigned)g_output_period);
        break;

    case CMD_SET_DEBUG:
        g_telem_debug = cmd->debug;
        SendResponse("ok:debug=%u\r\n", (unsigned)g_telem_debug);
        break;

    case CMD_SCAN: {
        float s = cmd->scan_start, e = cmd->scan_end, step = cmd->scan_step;
        uint16_t d = cmd->scan_delay_ms;
        int8_t inf = cmd->scan_infinite_dir;
        /* Зигзаг: start < end, step > 0, delay > 0. Бесконечное: step > 0, delay > 0 */
        if (step <= 0 || d == 0) {
            SendResponse("err:scan\r\n");
            break;
        }
        if (inf == 0 && s >= e) {
            SendResponse("err:scan\r\n");
            break;
        }
        g_cont_dir       = 0;
        g_scan_st        = SCAN_MOVING;
        g_target_deg     = s;
        g_scan_cur       = s;
        g_scan_start     = s;
        g_scan_end       = e;
        g_scan_step      = step;
        g_scan_delay_ms  = d;
        g_scan_delay_cnt = 0;
        g_scan_dir       = inf ? inf : 1;
        g_scan_inf       = inf;
        g_homing         = 0;
        PID_Reset(&g_pid);
        HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
        if (inf)
            SendResponse("ok:scan=%.2f,%c,%.2f,%u\r\n",
                (double)s, (inf > 0) ? '+' : '-', (double)step, (unsigned)d);
        else
            SendResponse("ok:scan=%.2f,%.2f,%.2f,%u\r\n",
                (double)s, (double)e, (double)step, (unsigned)d);
        break;
    }

    case CMD_STOP:
        g_cont_dir = 0;
        g_scan_inf = 0;
        g_scan_st  = SCAN_IDLE;
        tmc2209_motor_stop();
        g_target_deg = (g_mode == MODE_CL) ? COUNTS_TO_DEG_D(g_enc_counts) : g_ol_pos;
        Control_Reset();
        g_homing   = 0;
        HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
        SendResponse("ok:stop\r\n");
        break;

    case CMD_SET_VMAX:
        if (cmd->vmax < SPEED_MIN_DEG_S || cmd->vmax > (float)MAX_SPEED_DEG_S) {
            SendResponse("err:bad arg (v=%g..%g)\r\n",
                         (double)SPEED_MIN_DEG_S, (double)MAX_SPEED_DEG_S);
            break;
        }
        g_vmax_deg_s = cmd->vmax;
        /* PID должен насыщаться на новом пределе, иначе clamp его обгонит */
        g_pid.output_max =  vmax_per_tick();
        g_pid.output_min = -vmax_per_tick();
        SendResponse("ok:v=%.1f\r\n", (double)g_vmax_deg_s);
        break;

    case CMD_SET_ACCEL:
        if (cmd->accel < 0.0f || cmd->accel > ACCEL_MAX_DEG_S2) {
            SendResponse("err:bad arg (a=0..%g)\r\n", (double)ACCEL_MAX_DEG_S2);
            break;
        }
        g_accel_deg_s2 = cmd->accel;
        SendResponse("ok:a=%.1f\r\n", (double)g_accel_deg_s2);
        break;

    case CMD_SET_IRUN: {
        tmc2209_motor_config_t cfg; tmc2209_motor_get_config(&cfg);
        int r = tmc2209_motor_set_current(cmd->irun_ma, cfg.hold_ma);
        if      (r ==  0) SendResponse("ok:irun=%u\r\n", (unsigned)cmd->irun_ma);
        else if (r == -1) SendResponse("err:not ready\r\n");
        else if (r == -2) SendResponse("err:bad arg\r\n");
        else              SendResponse("err:apply failed\r\n");
        break;
    }

    case CMD_SET_IHOLD: {
        tmc2209_motor_config_t cfg; tmc2209_motor_get_config(&cfg);
        int r = tmc2209_motor_set_current(cfg.run_ma, cmd->ihold_ma);
        if      (r ==  0) SendResponse("ok:ihold=%u\r\n", (unsigned)cmd->ihold_ma);
        else if (r == -1) SendResponse("err:not ready\r\n");
        else if (r == -2) SendResponse("err:bad arg\r\n");
        else              SendResponse("err:apply failed\r\n");
        break;
    }

    case CMD_SET_ICUR: {
        int r = tmc2209_motor_set_current(cmd->irun_ma, cmd->ihold_ma);
        if      (r ==  0) SendResponse("ok:icur=%u,%u\r\n",
                              (unsigned)cmd->irun_ma, (unsigned)cmd->ihold_ma);
        else if (r == -1) SendResponse("err:not ready\r\n");
        else if (r == -2) SendResponse("err:bad arg\r\n");
        else              SendResponse("err:apply failed\r\n");
        break;
    }

    case CMD_SET_MSTEP: {
        /* Смена микрошагов безопасна при остановленном моторе */
        if (tmc2209_motor_is_moving()) {
            SendResponse("err:busy stop motor first\r\n");
            break;
        }
        int r = tmc2209_motor_set_microsteps(cmd->microsteps);
        if (r == 0) {
            /* Драйвер принял новый микрошаг — пересчитываем реальный масштаб
             * градус↔шаг (иначе snap/OL считались бы по compile-time значению). */
            g_deg_per_step = 360.0f / ((float)MOTOR_FULL_STEPS_REV * (float)cmd->microsteps);
            SendResponse("ok:mstep=%u\r\n", (unsigned)cmd->microsteps);
        }
        else if (r == -1) SendResponse("err:not ready\r\n");
        else if (r == -2) SendResponse("err:bad arg (1/2/4/8/16/32/64/128/256)\r\n");
        else              SendResponse("err:apply failed\r\n");
        break;
    }

    case CMD_GET_MCFG: {
        tmc2209_motor_config_t cfg;
        tmc2209_motor_get_config(&cfg);
        SendResponse("mode=%s run=%u hold=%u microsteps=%u ready=%u\r\n",
            (cfg.mode == TMC2209_MOTOR_CONTROL_STEP_DIR) ? "STEP_DIR" : "UART",
            (unsigned)cfg.run_ma, (unsigned)cfg.hold_ma,
            (unsigned)cfg.microsteps, (unsigned)cfg.ready);
        break;
    }

    case CMD_DIAG:
        /* Проверка спреда валидна только на неподвижном вале */
        if (g_cont_dir != 0 || g_scan_st != SCAN_IDLE || tmc2209_motor_is_moving()) {
            SendResponse("err:busy stop motor first\r\n");
            break;
        }
        EncDiag_Begin(&g_rediag);
        SendResponse("ok:diag\r\n");
        /* Результат придёт отдельной строкой enc:ok / err:enc через ~16 мс */
        break;

    case CMD_UNKNOWN:
        SendResponse("err:unknown\r\n");
        break;

    default:
        break;
    }
}

/* --- BSP --- */

static void BSP_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LED_PIN; gpio.Mode = GPIO_MODE_OUTPUT_PP; gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    gpio.Pin = SYNC_OUT_PIN; gpio.Mode = GPIO_MODE_OUTPUT_PP; gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SYNC_OUT_PORT, &gpio);
    HAL_GPIO_WritePin(SYNC_OUT_PORT, SYNC_OUT_PIN, GPIO_PIN_RESET);

    gpio.Pin = SYNC_IN_PIN; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(SYNC_IN_PORT, &gpio);
}

static void PollTimer_Init(void)
{
    htim_poll.Instance               = TIM2;
    htim_poll.Init.Prescaler         = (TIM2_CLK_HZ / 10000U) - 1;
    htim_poll.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim_poll.Init.Period            = (POLL_INTERVAL_MS * 10U) - 1;
    htim_poll.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_poll.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim_poll);

    HAL_NVIC_SetPriority(TIM2_IRQn, IRQ_PRIO_TIM_POLL, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
    if (htim->Instance == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    if (h->Instance == UART_INSTANCE || h->Instance == UART3_INSTANCE) {
        UART_CommandMspInit(h);
        return;
    }
    tmc2209_port_stm32_hal_uart_msp_init(h);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *h)
{
    if (h->Instance == UART_INSTANCE || h->Instance == UART3_INSTANCE) {
        UART_CommandMspDeInit(h);
        return;
    }
    tmc2209_port_stm32_hal_uart_msp_deinit(h);
}

/* Тактирование от HSI (внутренний генератор 8 МГц), кварца нет */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;  /* HSI/2 = 4 МГц */
    osc.PLL.PLLMUL          = RCC_PLL_MUL12;           /* 4 * 12 = 48 МГц */
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                          RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1);
}

void SysTick_Handler(void) { HAL_IncTick(); }

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim_poll);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
        Sample_ISR();
    else if (htim->Instance == TIM4)
        tmc2209_motor_tim4_period_elapsed();
}
