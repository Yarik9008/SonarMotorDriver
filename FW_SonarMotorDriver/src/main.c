/* main.c — Позиционное управление шаговым двигателем (PID + BiSS-C энкодер).
 *
 * Основные режимы: Closed-Loop (PID по энкодеру), Open-Loop (при сбое энкодера).
 * Команды через USB CDC и UART: en/dis, t=X, kp/ki/kd=X, scan, stop.
 * Таймер TIM2: 1 кГц — опрос энкодера, PID, телеметрия.
 */

#include "stm32f1xx_hal.h"
#include "board.h"
#include "biss_c.h"
#include "usb_cdc.h"
#include "tmc2209.h"
#include "pid.h"
#include "cmd_parser.h"
#include "uart.h"
#include <stdio.h>
#include <stdarg.h>

/* --- Прототипы --- */

static void SystemClock_Config(void);
static void BSP_Init(void);
static void PollTimer_Init(void);
static void ProcessCommand(const Cmd_Result *cmd);
static void SendResponse(const char *fmt, ...);
static void PollCommands(void);
static void Encoder_Accumulate(const BiSS_Reading *rd);
static void Mode_Update(uint8_t enc_ok);
static void MotorControl_Tick(uint8_t enc_ok);
static void Telemetry_Tick(uint8_t enc_ok, BiSS_Status st);
static void Heartbeat_Tick(void);
static void Scan_Tick(void);

/* --- Константы --- */

#define DEG_PER_STEP     (360.0f / (float)MOTOR_STEPS_PER_REV)
#define MAX_DEG_PER_TICK ((float)MAX_SPEED_DEG_S / (float)POLL_FREQ_HZ)
#define DT_S             (1.0f / (float)POLL_FREQ_HZ)
#define COUNTS_TO_DEG(c) ((float)(c) * 360.0f / (float)ENCODER_COUNTS_REV)

static inline int32_t DegToSteps(float deg)
{ return (int32_t)(deg / DEG_PER_STEP); }

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

/* --- Периферия --- */

static TIM_HandleTypeDef  htim_poll;
static IWDG_HandleTypeDef hiwdg;

/* --- DWT-задержки (блокирующие, только для HAL/USB internals) --- */

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

/* --- Неблокирующая стейт-машина инициализации --- */

typedef enum {
    INIT_USB_CDC, INIT_UART, INIT_USB_ENUM_WAIT,
    INIT_BISS_INIT, INIT_BISS_WAIT, INIT_MOTOR_DRIVER,
    INIT_POLL_TIMER, INIT_IWDG, INIT_DONE
} InitState;

static InitState s_init = INIT_USB_CDC;
static uint32_t  s_init_t0;

static uint8_t Init_Poll(void)
{
    switch (s_init) {

    case INIT_USB_CDC:
        USB_CDC_Init();
        s_init = INIT_UART;
        break;

    case INIT_UART:
        UART_Init();
        s_init_t0 = DWT->CYCCNT;
        s_init = INIT_USB_ENUM_WAIT;
        break;

    case INIT_USB_ENUM_WAIT:
        if ((DWT->CYCCNT - s_init_t0) >= DWT_MS_CYCLES(USB_ENUM_DELAY_MS))
            s_init = INIT_BISS_INIT;
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
        if ((DWT->CYCCNT - s_init_t0) >= DWT_MS_CYCLES(ENCODER_STARTUP_MS))
            s_init = INIT_MOTOR_DRIVER;
        break;

    case INIT_MOTOR_DRIVER:
        if (TMC2209_Init() == 0) {
            s_init = INIT_POLL_TIMER;
        }
        /* Если -1, остаёмся здесь: init motor subsystem failed */
        break;

    case INIT_POLL_TIMER:
        PollTimer_Init();
        HAL_TIM_Base_Start(&htim_poll);
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

/* --- Состояние --- */

typedef enum { MODE_CL = 0, MODE_OL = 1 } CtrlMode;

static PID_State g_pid = {
    .kp = PID_KP_DEFAULT, .ki = PID_KI_DEFAULT, .kd = PID_KD_DEFAULT,
    .integral = 0, .prev_error = 0,
    .output_min = -MAX_DEG_PER_TICK, .output_max = MAX_DEG_PER_TICK,
    .initialized = 0
};

static float    g_target_deg     = 0;
static uint8_t  g_enabled        = 0;
static uint16_t g_output_period  = OUTPUT_PERIOD_MS_DEFAULT;
static uint8_t  g_telem_debug    = TELEMETRY_DEBUG_DEFAULT;

static uint32_t g_enc_raw_prev   = 0xFFFFFFFF;
static int64_t  g_enc_counts     = 0;

static float    g_cl_accum       = 0;
static float    g_last_ctrl      = 0;

static CtrlMode g_mode           = MODE_CL;
static uint32_t g_enc_fail_cnt   = 0;
static float    g_ol_pos         = 0;
static uint8_t  g_was_outside_db = 0;
static uint8_t  g_homing         = 0;
static int8_t   g_cont_dir       = 0;

/* --- Сканирование --- */

typedef enum { SCAN_IDLE, SCAN_MOVING, SCAN_DELAY } ScanState;

static ScanState g_scan_st       = SCAN_IDLE;
static float     g_scan_cur      = 0;
static float     g_scan_start    = 0;
static float     g_scan_end      = 0;
static float     g_scan_step     = 0;
static uint16_t  g_scan_delay_ms = 0;
static uint16_t  g_scan_delay_cnt= 0;
static int8_t    g_scan_dir      = 1;
static int8_t    g_scan_inf      = 0;

static void DoSteps(int32_t steps)
{
    TMC2209_MoveSteps(steps);
}

/* --- MAIN --- */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    BSP_Init();
    Delay_Init();

    BiSS_Reading rd;
    BiSS_Status  st        = BISS_ERR_SPI;
    uint8_t      dma_pend  = 0;
    uint8_t      enc_ok    = 0;
    uint8_t      init_done = 0;

    while (1) {
        if (!init_done) {
            if (Init_Poll()) {
                st = BiSS_Read(&rd);
                if (st == BISS_OK || st == BISS_ERR_WARNING)
                    Encoder_Accumulate(&rd);
                BiSS_StartRead();
                dma_pend  = 1;
                enc_ok    = (g_enc_raw_prev != 0xFFFFFFFF);
                init_done = 1;
            }
            USB_CDC_Task();
            UART_Task();
            continue;
        }

        USB_CDC_Task();
        UART_Task();
        TMC2209_Task();
        PollCommands();

        if (!__HAL_TIM_GET_FLAG(&htim_poll, TIM_FLAG_UPDATE))
            continue;
        __HAL_TIM_CLEAR_FLAG(&htim_poll, TIM_FLAG_UPDATE);

        if (dma_pend && BiSS_IsReady()) {
            st = BiSS_GetResult(&rd);
            enc_ok = (st == BISS_OK || st == BISS_ERR_WARNING);
            if (enc_ok) Encoder_Accumulate(&rd);
            dma_pend = 0;
        }
        if (!dma_pend) {
            BiSS_StartRead();
            dma_pend = 1;
        }

        Mode_Update(enc_ok);
        MotorControl_Tick(enc_ok);
        Scan_Tick();
        Telemetry_Tick(enc_ok, st);
        Heartbeat_Tick();
        HAL_IWDG_Refresh(&hiwdg);
    }
}

/* --- Многооборотная позиция --- */

static void Encoder_Accumulate(const BiSS_Reading *rd)
{
    if (g_enc_raw_prev == 0xFFFFFFFF) {
        g_enc_counts   = (int64_t)rd->position;
        g_enc_raw_prev = rd->position;
        float pos = COUNTS_TO_DEG(g_enc_counts);
        g_target_deg = pos + shortest_path_err(STARTUP_TARGET_OFFSET_DEG, pos);
        g_ol_pos     = pos;
        g_homing     = 1;
        g_enabled    = 1;
        TMC2209_SetEnabled(1);
        return;
    }

    int32_t delta = (int32_t)rd->position - (int32_t)g_enc_raw_prev;
    if (delta >  (int32_t)(ENCODER_COUNTS_REV / 2)) delta -= (int32_t)ENCODER_COUNTS_REV;
    if (delta < -(int32_t)(ENCODER_COUNTS_REV / 2)) delta += (int32_t)ENCODER_COUNTS_REV;
    g_enc_counts   += delta;
    g_enc_raw_prev  = rd->position;
}

/* --- CL ↔ OL --- */

static void Mode_Update(uint8_t enc_ok)
{
    if (enc_ok) {
        g_enc_fail_cnt = 0;
        if (g_mode == MODE_OL) {
            g_mode = MODE_CL;
            PID_Reset(&g_pid);
            g_cl_accum = 0;
        }
    } else {
        g_enc_fail_cnt++;
        if (g_mode == MODE_CL && g_enc_fail_cnt >= (ENCODER_FAIL_MS / POLL_INTERVAL_MS)) {
            g_mode = MODE_OL;
            g_ol_pos = COUNTS_TO_DEG(g_enc_counts);
        }
    }
}

/* --- Управление мотором --- */

static void MotorControl_Tick(uint8_t enc_ok)
{
    if (!g_enabled) {
        TMC2209_Stop();
        g_last_ctrl = 0;
        return;
    }
    g_last_ctrl = 0;

    if (g_cont_dir != 0) {
        int32_t steps = g_cont_dir * (int32_t)MAX_STEPS_PER_POLL;
        DoSteps(steps);
        float moved  = (float)steps * DEG_PER_STEP;
        g_last_ctrl  = moved;
        g_ol_pos    += moved;
        g_target_deg+= moved;
        if (g_ol_pos > 1e7f || g_ol_pos < -1e7f) {
            g_ol_pos = g_target_deg = 0;
            g_enc_counts = (int64_t)g_enc_raw_prev;
        }
        return;
    }

    if (g_mode == MODE_CL && enc_ok) {
        float pos = COUNTS_TO_DEG(g_enc_counts);
        float err = g_target_deg - pos;

        if (err > PID_DEADBAND_DEG || err < -PID_DEADBAND_DEG) {
            g_was_outside_db = 1;
            float pid = PID_Update(&g_pid, err, DT_S);
            g_cl_accum = clampf(g_cl_accum + pid, -MAX_DEG_PER_TICK, MAX_DEG_PER_TICK);
            int32_t steps = clampi(DegToSteps(g_cl_accum),
                                   -(int32_t)MAX_STEPS_PER_POLL, (int32_t)MAX_STEPS_PER_POLL);
            if (steps) {
                if (g_scan_st == SCAN_MOVING)
                    HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
                DoSteps(steps);
                g_cl_accum  -= (float)steps * DEG_PER_STEP;
                g_last_ctrl  = (float)steps * DEG_PER_STEP;
            } else {
                g_last_ctrl = pid;
            }
        } else {
            if (g_was_outside_db) {
                if (err > 0.02f || err < -0.02f) {
                    int32_t snap = (err > 0) ? 1 : -1;
                    if (g_scan_st == SCAN_MOVING)
                        HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
                    DoSteps(snap);
                    g_last_ctrl = (float)snap * DEG_PER_STEP;
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
            TMC2209_Stop();
            PID_Reset(&g_pid);
            g_cl_accum = 0;
        }

    } else if (g_mode == MODE_OL) {
        float err = g_target_deg - g_ol_pos;

        if (err > PID_DEADBAND_DEG || err < -PID_DEADBAND_DEG) {
            err = clampf(err, -MAX_DEG_PER_TICK, MAX_DEG_PER_TICK);
            int32_t steps = clampi(DegToSteps(err),
                                   -(int32_t)MAX_STEPS_PER_POLL, (int32_t)MAX_STEPS_PER_POLL);
            if (steps) {
                if (g_scan_st == SCAN_MOVING)
                    HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
                DoSteps(steps);
                g_ol_pos    += (float)steps * DEG_PER_STEP;
                g_last_ctrl  = (float)steps * DEG_PER_STEP;
            } else {
                g_last_ctrl = err;
            }
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
            TMC2209_Stop();
        }
    }
}

/* --- Телеметрия --- */

static uint32_t g_dropped_tx = 0;

static void TransmitAll(const uint8_t *buf, uint16_t len)
{
    if (USB_CDC_IsConnected()) {
        if (USB_CDC_Transmit(buf, len) != 0) {
            g_dropped_tx++;
        }
    }
    if (UART_Transmit(buf, len) != 0) {
        g_dropped_tx++;
    }
}

static void Telemetry_Tick(uint8_t enc_ok, BiSS_Status st)
{
    static uint16_t cnt = 0;
    if (g_output_period == 0) return;
    if (++cnt < g_output_period) return;
    cnt = 0;

    char buf[128];
    float deg = enc_ok ? COUNTS_TO_DEG(g_enc_counts) : g_ol_pos;
    uint8_t ec = enc_ok ? ERR_OK : (uint8_t)st;
    int len;

    if (g_telem_debug)
        len = snprintf(buf, sizeof(buf),
            "cp:%.2f,tp:%.2f,pe:%.2f,u:%.4f,m:%s,ec:%u,kp:%.4f,ki:%.4f,kd:%.4f,drp:%lu\r\n",
            (double)deg, (double)g_target_deg, (double)(g_target_deg - deg),
            (double)g_last_ctrl, (g_mode == MODE_CL) ? "cl" : "ol",
            (unsigned)ec, (double)g_pid.kp, (double)g_pid.ki, (double)g_pid.kd, g_dropped_tx);
    else
        len = snprintf(buf, sizeof(buf), "cp:%.2f,ec:%u\r\n", (double)deg, (unsigned)ec);

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

/* --- Сканирование --- */

static void Scan_Tick(void)
{
    if (g_scan_st != SCAN_DELAY) return;

    if (++g_scan_delay_cnt < g_scan_delay_ms) return;

    float next;
    if (g_scan_inf != 0) {
        next = g_scan_cur + (float)g_scan_inf * g_scan_step;
        if (next > 1e7f || next < -1e7f) {
            float off = g_scan_cur;
            g_scan_cur   -= off;
            next         -= off;
            g_ol_pos     -= off;
            g_target_deg -= off;
            g_enc_counts -= (int64_t)(off / 360.0f * (float)ENCODER_COUNTS_REV);
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
    PID_Reset(&g_pid);
    g_cl_accum = 0;
}

/* --- Команды --- */

static void PollCommands(void)
{
    char line[64];
    Cmd_Result cmd;
    if (USB_CDC_ReadLine(line, sizeof(line)) > 0 && Cmd_Parse(line, &cmd))
        ProcessCommand(&cmd);
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
        g_cont_dir = 0;
        TMC2209_SetEnabled(1);
        PID_Reset(&g_pid);
        g_cl_accum   = 0;
        g_target_deg = (g_mode == MODE_CL) ? COUNTS_TO_DEG(g_enc_counts) : g_ol_pos;
        g_homing     = 0;
        SendResponse("ok:en\r\n");
        break;

    case CMD_DISABLE:
        g_enabled  = 0;
        g_cont_dir = 0;
        TMC2209_Stop();
        TMC2209_SetEnabled(0);
        SendResponse("ok:dis\r\n");
        break;

    case CMD_SET_TARGET:
        g_cont_dir   = 0;
        g_target_deg = cmd->target;
        g_homing     = 0;
        PID_Reset(&g_pid);
        g_cl_accum = 0;
        SendResponse("ok:t=%.2f\r\n", (double)g_target_deg);
        break;

    case CMD_CONTINUOUS:
        g_cont_dir = cmd->continuous_dir;
        g_scan_st  = SCAN_IDLE;
        g_homing   = 0;
        PID_Reset(&g_pid);
        g_cl_accum = 0;
        if (!g_enabled) { g_enabled = 1; TMC2209_SetEnabled(1); }
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
        if ((inf == 0 && (s > e || step <= 0)) || (inf != 0 && step <= 0)) {
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
        g_cl_accum = 0;
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
        TMC2209_Stop();
        g_target_deg = (g_mode == MODE_CL) ? COUNTS_TO_DEG(g_enc_counts) : g_ol_pos;
        PID_Reset(&g_pid);
        g_cl_accum = 0;
        g_homing   = 0;
        HAL_GPIO_WritePin(SYNC_PORT, SYNC_PIN, GPIO_PIN_RESET);
        SendResponse("ok:stop\r\n");
        break;

    case CMD_SET_IRUN: {
        TMC2209_Config cfg; TMC2209_GetConfig(&cfg);
        int r = TMC2209_SetCurrent(cmd->irun_ma, cfg.hold_ma);
        if      (r ==  0) SendResponse("ok:irun=%u\r\n", (unsigned)cmd->irun_ma);
        else if (r == -1) SendResponse("err:not ready\r\n");
        else if (r == -2) SendResponse("err:bad arg\r\n");
        else              SendResponse("err:apply failed\r\n");
        break;
    }

    case CMD_SET_IHOLD: {
        TMC2209_Config cfg; TMC2209_GetConfig(&cfg);
        int r = TMC2209_SetCurrent(cfg.run_ma, cmd->ihold_ma);
        if      (r ==  0) SendResponse("ok:ihold=%u\r\n", (unsigned)cmd->ihold_ma);
        else if (r == -1) SendResponse("err:not ready\r\n");
        else if (r == -2) SendResponse("err:bad arg\r\n");
        else              SendResponse("err:apply failed\r\n");
        break;
    }

    case CMD_SET_ICUR: {
        int r = TMC2209_SetCurrent(cmd->irun_ma, cmd->ihold_ma);
        if      (r ==  0) SendResponse("ok:icur=%u,%u\r\n",
                              (unsigned)cmd->irun_ma, (unsigned)cmd->ihold_ma);
        else if (r == -1) SendResponse("err:not ready\r\n");
        else if (r == -2) SendResponse("err:bad arg\r\n");
        else              SendResponse("err:apply failed\r\n");
        break;
    }

    case CMD_SET_MSTEP: {
        /* Смена микрошагов безопасна при остановленном моторе */
        if (TMC2209_IsMoving()) {
            SendResponse("err:busy stop motor first\r\n");
            break;
        }
        int r = TMC2209_SetMicrosteps(cmd->microsteps);
        if      (r ==  0) SendResponse("ok:mstep=%u\r\n", (unsigned)cmd->microsteps);
        else if (r == -1) SendResponse("err:not ready\r\n");
        else if (r == -2) SendResponse("err:bad arg (1/2/4/8/16/32/64/128/256)\r\n");
        else              SendResponse("err:apply failed\r\n");
        break;
    }

    case CMD_GET_MCFG: {
        TMC2209_Config cfg;
        TMC2209_GetConfig(&cfg);
        SendResponse("mode=%s run=%u hold=%u microsteps=%u ready=%u\r\n",
            (cfg.mode == TMC2209_CONTROL_MODE_STEP_DIR) ? "STEP_DIR" : "UART",
            (unsigned)cfg.run_ma, (unsigned)cfg.hold_ma,
            (unsigned)cfg.microsteps, (unsigned)cfg.ready);
        break;
    }

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
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL          = RCC_PLL_MUL12;
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                          RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1);

    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    pclk.UsbClockSelection    = RCC_USBCLKSOURCE_PLL;
    HAL_RCCEx_PeriphCLKConfig(&pclk);
}

void SysTick_Handler(void) { HAL_IncTick(); }
