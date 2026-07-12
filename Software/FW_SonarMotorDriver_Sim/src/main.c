/**
 * @file main.c
 * @brief Прошивка-имитатор SonarMotorDriver.
 *
 * Полностью повторяет UART-протокол основной прошивки (команды и телеметрия
 * на USART1 PA9/PA10, 115200 8N1), но НЕ содержит реального функционала:
 * - нет обмена с энкодером LENZ IRS (SPI/BiSS C не инициализируются);
 * - нет управления TMC2209 (USART2, TIM4, STEP/DIR не используются).
 *
 * Вместо этого движение моделируется программно: виртуальная позиция плавно
 * идёт к цели с профилем скорости/ускорения (v=/a=), как open-loop ветка
 * MotorControl_Tick основной прошивки (g_ol_pos += v — без квантования в шаги).
 * Телеметрия всегда показывает штатную работу: ec:0, m:cl.
 *
 * Поддерживаются все команды основной прошивки: en, dis, t=, t=+/-, kp/ki/kd=,
 * op=, debug=, scan=, stop, irun, ihold, icur, mstep, mcfg, diag — с теми же
 * ответами. При старте, как и основная прошивка, отправляет строку диагностики
 * энкодера (у имитатора — всегда успешную): enc:ok n=16/16 spread=0.000 pos=<...>.
 * Команда diag отвечает ok:diag и той же строкой enc:ok.
 *
 * Синхронизация (полноценный функционал, одинаковый с основной прошивкой):
 * - SYNC_OUT (PB9), как в основной: LOW — движение к точке скана,
 *   HIGH — точка достигнута (по модели движения);
 * - SYNC_IN (PB12) — внешний триггер: опрос 1 кГц, детектор фронта LOW→HIGH.
 *   Команда sync=N выбирает источник перехода к следующей точке скана:
 *   0 — таймер delay (штатно, по умолчанию), 1 — фронт SYNC_IN,
 *   2 — фронт SYNC_IN либо delay как тайм-аут. Учитываются только фронты,
 *   пришедшие после выхода SYNC_OUT в HIGH (точка достигнута).
 *   Команда sync выводит состояние: sync=<N> in=<0|1> out=<0|1> n=<фронтов>.
 */

#include "stm32f1xx_hal.h"
#include "board.h"
#include "cmd_parser.h"
#include "uart.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

/* --- Прототипы --- */

static void SystemClock_Config(void);
static void BSP_Init(void);
static void PollTimer_Init(void);
static void ProcessCommand(const Cmd_Result *cmd);
static void SendResponse(const char *fmt, ...);
static void PollCommands(void);
static void MotorControl_Tick(void);
static void Telemetry_Tick(void);
static void Heartbeat_Tick(void);
static void Scan_Tick(void);
static void SyncIn_Tick(void);

/* --- Константы (те же формулы, что в основной прошивке) --- */

#define DEG_PER_STEP     (360.0f / (float)MOTOR_STEPS_PER_REV)

/* Стартовая виртуальная позиция: ненулевая, чтобы после сброса был виден
 * «хоминг» в 0°, как у реальной платы. */
#define SIM_START_POS_DEG 37.5f

static inline int32_t DegToSteps(float deg)
{ return (int32_t)(deg / DEG_PER_STEP); }

static inline float clampf(float v, float lo, float hi)
{ return (v < lo) ? lo : (v > hi) ? hi : v; }

static inline float shortest_path_err(float target, float pos)
{
    float err = target - pos;
    while (err > 180.0f)   err -= 360.0f;
    while (err <= -180.0f) err += 360.0f;
    return err;
}

/* --- Периферия --- */

static TIM_HandleTypeDef  htim_poll;
static IWDG_HandleTypeDef hiwdg;

/* --- Состояние имитатора --- */

static float    g_sim_pos_deg    = SIM_START_POS_DEG; ///< Виртуальная позиция «энкодера»
static float    g_target_deg     = 0;           ///< Целевая позиция (градусы)
static uint8_t  g_enabled        = 1;           ///< Флаг разрешения движения
static uint16_t g_output_period  = OUTPUT_PERIOD_MS_DEFAULT; ///< Период телеметрии
static uint8_t  g_telem_debug    = TELEMETRY_DEBUG_DEFAULT;  ///< Флаг расширенной телеметрии

static float    g_last_ctrl      = 0;           ///< Последнее «управляющее воздействие» (град/тик)
static uint8_t  g_was_outside_db = 0;           ///< Флаг выхода из мертвой зоны (для доводки)
static int8_t   g_cont_dir       = 0;           ///< Направление непрерывного вращения (t+/t-)

/* Хранимые параметры PID: на модель не влияют, но отображаются в телеметрии
 * debug=1 и подтверждаются в ответах — как у основной прошивки. */
static float    g_kp = PID_KP_DEFAULT;
static float    g_ki = PID_KI_DEFAULT;
static float    g_kd = PID_KD_DEFAULT;

/* Профиль движения (v= и a=) — влияет на модель, как у основной прошивки */
static float    g_vmax_deg_s   = SPEED_DEFAULT_DEG_S;  ///< Предел скорости, град/с
static float    g_accel_deg_s2 = ACCEL_DEFAULT_DEG_S2; ///< Предел ускорения, град/с² (0=выкл)
static float    g_vel_cmd      = 0;                    ///< Текущая слew-скорость, град/тик

static inline float vmax_per_tick(void)
{ return g_vmax_deg_s / (float)POLL_FREQ_HZ; }

/* Ограничитель скорости/ускорения — та же логика, что в основной прошивке */
static float Motion_Limit(float v_des, float err, uint8_t have_err)
{
    float vmax = vmax_per_tick();
    v_des = clampf(v_des, -vmax, vmax);

    if (g_accel_deg_s2 > 0) {
        const float dt = 1.0f / (float)POLL_FREQ_HZ;
        if (have_err) {
            float ae = g_accel_deg_s2 * (err < 0 ? -err : err);
            float vbrake = sqrtf(2.0f * ae) * dt;
            v_des = clampf(v_des, -vbrake, vbrake);
        }
        float dv = g_accel_deg_s2 * dt * dt;
        g_vel_cmd += clampf(v_des - g_vel_cmd, -dv, dv);
    } else {
        g_vel_cmd = v_des;
    }
    return g_vel_cmd;
}

/* Хранимая «конфигурация TMC2209» для irun/ihold/icur/mstep/mcfg */
static uint16_t g_run_ma     = TMC2209_IRUN_MA;
static uint16_t g_hold_ma    = TMC2209_IHOLD_MA;
static uint16_t g_microsteps = TMC2209_MICROSTEPS;

/**
 * @brief Состояние логики автоматического сканирования (как в оригинале).
 */
typedef enum {
    SCAN_IDLE,      ///< Сканирование не запущено
    SCAN_MOVING,    ///< Ожидание достижения целевой позиции
    SCAN_DELAY      ///< Ожидание в целевой позиции
} ScanState;

static ScanState g_scan_st       = SCAN_IDLE;
static float     g_scan_cur      = 0;
static float     g_scan_start    = 0;
static float     g_scan_end      = 0;
static float     g_scan_step     = 0;
static uint16_t  g_scan_delay_ms = 0;
static uint16_t  g_scan_delay_cnt= 0;
static int8_t    g_scan_dir      = 1;
static int8_t    g_scan_inf      = 0;

/**
 * @brief Источник перехода к следующей точке скана (команда sync=N).
 */
typedef enum {
    SYNC_ADV_TIMER = 0,     ///< Таймер delay из scan= (поведение основной прошивки)
    SYNC_ADV_EXT,           ///< Фронт LOW→HIGH на SYNC_IN (delay игнорируется)
    SYNC_ADV_EXT_TIMEOUT    ///< Фронт SYNC_IN либо delay как тайм-аут
} SyncAdvMode;

static uint8_t  g_sync_mode    = SYNC_ADV_TIMER; ///< Текущий режим (sync=)
static uint8_t  g_syncin_prev  = 0;   ///< Уровень SYNC_IN на прошлом тике
static uint8_t  g_syncin_trig  = 0;   ///< Одноразовый флаг «фронт после достижения точки»
static uint32_t g_syncin_edges = 0;   ///< Счётчик фронтов SYNC_IN с момента старта

/* «Мотор движется» для проверки busy в mstep/diag. В основной прошивке diag
 * занят при (g_cont_dir != 0 || g_scan_st != SCAN_IDLE || motor_is_moving()) —
 * скан считается движением даже в паузе между точками (вал в SCAN_DELAY). */
static uint8_t Sim_IsMoving(void)
{
    if (g_cont_dir != 0 || g_scan_st != SCAN_IDLE) return 1;
    if (!g_enabled) return 0;
    float err = g_target_deg - g_sim_pos_deg;
    return (err > PID_DEADBAND_DEG || err < -PID_DEADBAND_DEG) ? 1U : 0U;
}

/* «Диагностика энкодера» имитатора: всегда успешна, формат как у основной */
static void SendEncDiagOk(void)
{
    SendResponse("enc:ok n=%u/%u spread=0.000 pos=%.2f\r\n",
                 (unsigned)ENCODER_DIAG_SAMPLES, (unsigned)ENCODER_DIAG_SAMPLES,
                 (double)g_sim_pos_deg);
}

/* --- MAIN --- */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    BSP_Init();

    /* Неблокирующая инициализация: UART → таймер опроса → IWDG */
    while (UART_Init() != 0) { }

    /* «Диагностика энкодера»: у имитатора всегда успешна — сообщение как у
     * основной прошивки (enc:ok), чтобы хост-софт видел одинаковый старт. */
    SendEncDiagOk();

    /* «Первое чтение энкодера»: цель — дом (0° + офсет) кратчайшим путём */
    g_target_deg = g_sim_pos_deg +
                   shortest_path_err(STARTUP_TARGET_OFFSET_DEG, g_sim_pos_deg);

    PollTimer_Init();
    HAL_TIM_Base_Start(&htim_poll);

    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER;
    hiwdg.Init.Reload    = IWDG_RELOAD;
    HAL_IWDG_Init(&hiwdg);

    while (1) {
        UART_Task();
        PollCommands();

        if (!__HAL_TIM_GET_FLAG(&htim_poll, TIM_FLAG_UPDATE))
            continue;
        __HAL_TIM_CLEAR_FLAG(&htim_poll, TIM_FLAG_UPDATE);

        SyncIn_Tick();
        MotorControl_Tick();
        Scan_Tick();
        Telemetry_Tick();
        Heartbeat_Tick();
        HAL_IWDG_Refresh(&hiwdg);
    }
}

/* --- Модель движения (1 кГц) --- */

/**
 * @brief Расчет виртуального перемещения за тик.
 *
 * Модель повторяет open-loop ветку MotorControl_Tick основной прошивки:
 * скорость на тик даёт профиль Motion_Limit (v=/a=), а позиция интегрируется
 * плавно (g_sim_pos_deg += v) — как g_ol_pos += v в прошивке, без квантования
 * в микрошаги. Поэтому cp/u в телеметрии совпадают с реальной платой (там cp
 * приходит с 17-битного энкодера, а u = скомандованная скорость град/тик).
 * Доводка в deadband повторяет финальный доворот CL-ветки (снап DegToSteps).
 */
static void MotorControl_Tick(void)
{
    if (!g_enabled) {
        g_last_ctrl = 0;
        return;
    }
    g_last_ctrl = 0;

    if (g_cont_dir != 0) {
        /* Непрерывное вращение на пределе v= с рампой a= (как в основной) */
        float v = Motion_Limit((float)g_cont_dir * vmax_per_tick(), 0, 0);
        g_sim_pos_deg += v;
        g_target_deg  += v;
        g_last_ctrl    = v;
        if (g_sim_pos_deg > 1e7f || g_sim_pos_deg < -1e7f)
            g_sim_pos_deg = g_target_deg = 0;
        return;
    }

    float err = g_target_deg - g_sim_pos_deg;

    if (err > PID_DEADBAND_DEG || err < -PID_DEADBAND_DEG) {
        g_was_outside_db = 1;
        float v = Motion_Limit(err, err, 1);
        if (g_scan_st == SCAN_MOVING && v != 0.0f)
            HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
        g_sim_pos_deg += v;
        g_last_ctrl    = v;
    } else {
        /* Доводка до цели: финальный доворот CL-ветки основной прошивки — снап
         * на DegToSteps(err) садится в пределах одного микрошага от цели. */
        if (g_was_outside_db) {
            if (err > 0.02f || err < -0.02f) {
                int32_t snap = DegToSteps(err);
                if (snap == 0)
                    snap = (err > 0) ? 1 : -1;
                if (g_scan_st == SCAN_MOVING)
                    HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
                g_sim_pos_deg += (float)snap * DEG_PER_STEP;
                g_last_ctrl    = (float)snap * DEG_PER_STEP;
            }
            g_was_outside_db = 0;
        }
        if (g_scan_st == SCAN_MOVING) {
            HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_SET);
            g_scan_cur       = g_target_deg;
            g_scan_st        = SCAN_DELAY;
            g_scan_delay_cnt = 0;
            g_syncin_trig    = 0; /* фронты SYNC_IN до выхода SYNC_OUT в HIGH не считаются */
        }
        g_vel_cmd = 0;   /* цель достигнута — обнуляем скорость рампы (как Control_Reset) */
    }
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

static void Telemetry_Tick(void)
{
    static uint16_t cnt = 0;
    if (g_output_period == 0) return;
    /* Даём коротким CLI-ответам уйти целой строкой, не перемешивая их с телеметрией. */
    if (UART_TxPending()) return;
    /* При полной телеметрии (debug=1) используем не меньший период */
    uint16_t period = g_telem_debug ? (g_output_period >= OUTPUT_PERIOD_MS_DEBUG_MIN ? g_output_period : OUTPUT_PERIOD_MS_DEBUG_MIN) : g_output_period;
    if (++cnt < period) return;
    cnt = 0;

    char buf[160];
    float deg = g_sim_pos_deg;
    int len;

    if (g_telem_debug)
        len = snprintf(buf, sizeof(buf),
            "cp:%.2f,tp:%.2f,pe:%.2f,u:%.4f,m:%s,ec:%u,kp:%.4f,ki:%.4f,kd:%.4f,v:%.1f,a:%.1f,of:0,drp:%lu\r\n",
            (double)deg, (double)g_target_deg, (double)(g_target_deg - deg),
            (double)g_last_ctrl, "cl",
            (unsigned)ERR_OK, (double)g_kp, (double)g_ki, (double)g_kd,
            (double)g_vmax_deg_s, (double)g_accel_deg_s2, (unsigned long)g_dropped_tx);
    else
        len = snprintf(buf, sizeof(buf), "cp:%.2f,ec:%u\r\n", (double)deg, (unsigned)ERR_OK);

    if (len >= (int)sizeof(buf)) len = (int)sizeof(buf) - 1;  /* snprintf усёк */
    if (len > 0) TransmitAll((uint8_t *)buf, (uint16_t)len);
}

static void Heartbeat_Tick(void)
{
    static uint32_t cnt = 0;
    if (++cnt >= LED_TOGGLE_INTERVAL) {
        cnt = 0;
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }
}

/* --- Синхронизация: опрос SYNC_IN (1 кГц) --- */

/**
 * @brief Детектор фронта LOW→HIGH на SYNC_IN (импульс должен быть >= 1 мс).
 *
 * Каждый фронт увеличивает счётчик g_syncin_edges (диагностика, команда sync)
 * и взводит g_syncin_trig — триггер перехода к следующей точке скана в
 * режимах sync=1/2. Перед следующим фронтом линия должна вернуться в LOW.
 */
static void SyncIn_Tick(void)
{
    uint8_t lvl = (HAL_GPIO_ReadPin(SYNC_IN_PORT, SYNC_IN_PIN) == GPIO_PIN_SET);
    if (lvl && !g_syncin_prev) {
        g_syncin_edges++;
        g_syncin_trig = 1;
    }
    g_syncin_prev = lvl;
}

/* --- Сканирование (перенесено из основной прошивки) --- */

static void Scan_Tick(void)
{
    if (g_scan_st != SCAN_DELAY) return;

    /* Переход к следующей точке: по таймеру delay и/или фронту SYNC_IN (sync=) */
    uint8_t timer_ok = (g_scan_delay_ms != 0 &&
                        ++g_scan_delay_cnt >= g_scan_delay_ms);
    uint8_t go = 0;
    switch (g_sync_mode) {
    case SYNC_ADV_TIMER:       go = timer_ok;                  break;
    case SYNC_ADV_EXT:         go = g_syncin_trig;             break;
    case SYNC_ADV_EXT_TIMEOUT: go = g_syncin_trig || timer_ok; break;
    default:                   break;
    }
    if (!go) return;
    g_syncin_trig = 0;

    float next;
    if (g_scan_inf != 0) {
        next = g_scan_cur + (float)g_scan_inf * g_scan_step;
        if (next > 1e7f || next < -1e7f) {
            float off = g_scan_cur;
            g_scan_cur    -= off;
            next          -= off;
            g_sim_pos_deg -= off;
            g_target_deg  -= off;
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

/* Валидация микрошага: степень двойки в диапазоне 1..256
 * (как is_valid_microsteps в lib/tmc2209) */
static uint8_t sim_valid_microsteps(uint16_t ms)
{
    return ms > 0 && ms <= 256 && (ms & (ms - 1)) == 0;
}

static void ProcessCommand(const Cmd_Result *cmd)
{
    switch (cmd->type) {

    case CMD_ENABLE:
        g_enabled  = 1;
        g_cont_dir = 0;
        g_target_deg = g_sim_pos_deg;
        g_vel_cmd    = 0;
        SendResponse("ok:en\r\n");
        break;

    case CMD_DISABLE:
        g_enabled  = 0;
        g_cont_dir = 0;
        SendResponse("ok:dis\r\n");
        break;

    case CMD_SET_TARGET:
        g_cont_dir   = 0;
        g_target_deg = cmd->target;
        SendResponse("ok:t=%.2f\r\n", (double)g_target_deg);
        break;

    case CMD_CONTINUOUS:
        g_cont_dir = cmd->continuous_dir;
        g_scan_st  = SCAN_IDLE;
        if (!g_enabled) g_enabled = 1;
        SendResponse("ok:t=%c\r\n", (g_cont_dir > 0) ? '+' : '-');
        break;

    case CMD_SET_KP:
        g_kp = cmd->kp;
        SendResponse("ok:kp=%.4f\r\n", (double)g_kp);
        break;

    case CMD_SET_KI:
        g_ki = cmd->ki;
        SendResponse("ok:ki=%.4f\r\n", (double)g_ki);
        break;

    case CMD_SET_KD:
        g_kd = cmd->kd;
        SendResponse("ok:kd=%.4f\r\n", (double)g_kd);
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
        g_syncin_trig    = 0;  /* фронт, пришедший до старта скана, не считается */
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
        g_target_deg = g_sim_pos_deg;
        g_vel_cmd    = 0;
        g_syncin_trig = 0;
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

    case CMD_SET_IRUN:
        g_run_ma = cmd->irun_ma;
        SendResponse("ok:irun=%u\r\n", (unsigned)cmd->irun_ma);
        break;

    case CMD_SET_IHOLD:
        g_hold_ma = cmd->ihold_ma;
        SendResponse("ok:ihold=%u\r\n", (unsigned)cmd->ihold_ma);
        break;

    case CMD_SET_ICUR:
        g_run_ma  = cmd->irun_ma;
        g_hold_ma = cmd->ihold_ma;
        SendResponse("ok:icur=%u,%u\r\n",
            (unsigned)cmd->irun_ma, (unsigned)cmd->ihold_ma);
        break;

    case CMD_SET_MSTEP:
        /* Смена микрошагов «безопасна» только при остановленном моторе */
        if (Sim_IsMoving()) {
            SendResponse("err:busy stop motor first\r\n");
            break;
        }
        if (!sim_valid_microsteps(cmd->microsteps)) {
            SendResponse("err:bad arg (1/2/4/8/16/32/64/128/256)\r\n");
            break;
        }
        g_microsteps = cmd->microsteps;
        SendResponse("ok:mstep=%u\r\n", (unsigned)cmd->microsteps);
        break;

    case CMD_GET_MCFG:
        SendResponse("mode=%s run=%u hold=%u microsteps=%u ready=%u\r\n",
            "STEP_DIR",
            (unsigned)g_run_ma, (unsigned)g_hold_ma,
            (unsigned)g_microsteps, 1U);
        break;

    case CMD_DIAG:
        /* Как в основной прошивке: только на неподвижном вале */
        if (Sim_IsMoving()) {
            SendResponse("err:busy stop motor first\r\n");
            break;
        }
        SendResponse("ok:diag\r\n");
        SendEncDiagOk();
        break;

    case CMD_SET_SYNC:
        g_sync_mode = cmd->sync_mode;   /* 0..2 гарантирует парсер */
        SendResponse("ok:sync=%u\r\n", (unsigned)g_sync_mode);
        break;

    case CMD_GET_SYNC:
        SendResponse("sync=%u in=%u out=%u n=%lu\r\n",
            (unsigned)g_sync_mode,
            (unsigned)(HAL_GPIO_ReadPin(SYNC_IN_PORT, SYNC_IN_PIN) == GPIO_PIN_SET),
            (unsigned)(HAL_GPIO_ReadPin(SYNC_OUT_PORT, SYNC_OUT_PIN) == GPIO_PIN_SET),
            (unsigned long)g_syncin_edges);
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

    /* Безопасность: если имитатор прошит в реальную плату драйвера,
     * удерживаем TMC2209 выключенным (ENN = HIGH). Больше никакого
     * взаимодействия с драйвером нет. */
    gpio.Pin = ENABLE_PIN; gpio.Mode = GPIO_MODE_OUTPUT_PP; gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ENABLE_PORT, &gpio);
    HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, GPIO_PIN_SET);
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
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    UART_CommandMspInit(h);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *h)
{
    UART_CommandMspDeInit(h);
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
