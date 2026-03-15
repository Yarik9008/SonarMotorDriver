/**
 * @file tmc2209_motor.c
 * @brief Реализация фасада управления мотором.
 *
 * Модуль объединяет в себе управление чипом по UART (через tmc2209.c) и
 * генерацию импульсов STEP/DIR (через tmc2209_port_stm32_hal.c).
 *
 * Кэширование: для ускорения работы некоторые параметры (ток, микрошаг)
 * дублируются в локальных переменных s_run_ma, s_hold_ma и т.д. Также
 * реализован фоновый опрос статуса чипа (диагностика) с заданным интервалом.
 */

#include "tmc2209/tmc2209_motor.h"
#include "tmc2209/tmc2209.h"
#include "tmc2209/tmc2209_port_stm32_hal.h"
#include "board.h"
#include <stdint.h>

/* ---- Внутреннее состояние ---- */

static tmc2209_t          s_drv;        ///< Контекст чипа TMC2209
static UART_HandleTypeDef  s_huart;      ///< Дескриптор UART для связи с драйвером
static tmc2209_hal_ctx_t   s_hal;        ///< HAL-контекст порта
static TIM_HandleTypeDef   s_htim_step;  ///< Таймер для генерации импульсов STEP
static uint8_t            s_tmc_ready = 0; ///< Флаг успешной инициализации

/* Diagnostics */
#define DIAG_INTERVAL_MS   500U
static uint32_t            s_diag_last_ms = 0;
static tmc2209_drv_status_t s_diag_cached = {0};
static uint8_t             s_diag_valid   = 0;

/* Motor Configuration */
static uint16_t s_run_ma    = TMC2209_IRUN_MA;
static uint16_t s_hold_ma   = TMC2209_IHOLD_MA;
static uint16_t s_microsteps = TMC2209_MICROSTEPS;

/* STEP/DIR Backend State */
static uint8_t             g_pwm_running = 0;
static int8_t              g_cur_dir     = -1;
static uint32_t            g_cur_arr     = 0;

#define TICKS_PER_US     (TIM4_CLK_HZ / 1000000U)
#define PULSE_TICKS      (STEP_PULSE_US * TICKS_PER_US)
#define MIN_PERIOD_TICKS (PULSE_TICKS + 1U)

/* ---- Runtime & Task Management ---- */

static void tmc2209_motor_reset_runtime_state(void)
{
    /* Motor Backend (STEP/DIR) */
    g_pwm_running = 0;
    g_cur_dir     = -1;
    g_cur_arr     = 0;

    /* Config snapshot */
    s_run_ma      = TMC2209_IRUN_MA;
    s_hold_ma     = TMC2209_IHOLD_MA;
    s_microsteps  = TMC2209_MICROSTEPS;

    /* Diagnostics */
    s_diag_last_ms = 0;
    s_diag_cached  = (tmc2209_drv_status_t){0};
    s_diag_valid   = 0;
    
    /* Readiness */
    s_tmc_ready    = 0;
}

/* ---- Internal Helpers ---- */

static int map_result(tmc2209_result_t res)
{
    if (res == TMC2209_OK) return 0;
    if (res == TMC2209_ERR_INVALID_ARG) return -2;
    return -3;
}

static int tmc2209_init_backend(void)
{
    /* UART Init */
    s_huart.Instance          = TMC2209_UART;
    s_huart.Init.BaudRate     = TMC2209_UART_BAUDRATE;
    s_huart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_huart.Init.StopBits     = UART_STOPBITS_1;
    s_huart.Init.Parity       = UART_PARITY_NONE;
    s_huart.Init.Mode         = UART_MODE_TX_RX;
    s_huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_huart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&s_huart) != HAL_OK) return -1;

    /* Port Context */
    s_hal.huart       = &s_huart;
    s_hal.en_port     = ENABLE_PORT;
    s_hal.en_pin      = ENABLE_PIN;
    s_hal.sysclk_hz   = SYSCLK_HZ;
    s_hal.half_duplex = TMC2209_HALF_DUPLEX;
    s_hal.debug_fn    = NULL;

    /* Motor Backend info */
    s_htim_step.Instance = TIM4;
    s_htim_step.Init.Prescaler         = 0;
    s_htim_step.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim_step.Init.Period            = 999;
    s_htim_step.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim_step.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    s_hal.htim_step   = &s_htim_step;
    s_hal.tim_channel = TIM_CHANNEL_3;
    s_hal.step_port   = STEP_PORT;
    s_hal.step_pin    = STEP_PIN;
    s_hal.dir_port    = DIR_PORT;
    s_hal.dir_pin     = DIR_PIN;

    tmc2209_io_t io;
    tmc2209_port_stm32_hal_fill_io(&io, &s_hal);

    /* STEP/DIR HW Init via Port */
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    if (io.motor_hw_init(&s_hal) != 0) return -1;
#endif

    /* TMC2209 Chip Init */
    tmc2209_config_t cfg = TMC2209_DEFAULT_CONFIG;
    cfg.addr           = TMC2209_UART_ADDR;
    cfg.rsense         = TMC2209_RSENSE_OHM;
    cfg.irun_ma        = TMC2209_IRUN_MA;
    cfg.ihold_ma       = TMC2209_IHOLD_MA;
    cfg.microsteps     = TMC2209_MICROSTEPS;
    cfg.reply_delay_us = TMC2209_REPLY_DELAY_US;
    cfg.senddelay      = TMC2209_CFG_SENDDELAY;
    cfg.tpowerdown     = TMC2209_TPOWERDOWN;
    cfg.en_spreadcycle = TMC2209_SPREADCYCLE;

    tmc2209_result_t res = tmc2209_init(&s_drv, &cfg, &io);
    if (res == TMC2209_OK) {
        s_tmc_ready = 1;
        return 0;
    }
    return (int)res;
}

static void stepdir_steps_internal(int32_t steps)
{
    if (steps == 0) {
        tmc2209_motor_stop();
        return;
    }

    uint32_t n   = (uint32_t)(steps < 0 ? -steps : steps);
    int8_t   dir = (steps > 0) ? 1 : 0;

    if (dir != g_cur_dir) {
        s_drv.io.motor_set_dir(dir, s_drv.io.ctx);
        g_cur_dir = dir;
    }

    uint32_t period_us = 1000U / n;
    if (period_us < STEP_PULSE_US + 1U) period_us = STEP_PULSE_US + 1U;

    uint32_t arr = period_us * TICKS_PER_US;
    if (arr < MIN_PERIOD_TICKS) arr = MIN_PERIOD_TICKS;
    if (arr > 65535U)           arr = 65535U;

    uint32_t ccr = (PULSE_TICKS < arr) ? PULSE_TICKS : (arr / 2U);

    if (!g_pwm_running || arr != g_cur_arr) {
        s_drv.io.motor_set_rate(arr, ccr, s_drv.io.ctx);
        g_pwm_running = 1;
        g_cur_arr     = arr;
    }
}

/* ---- Initialization ---- */

int tmc2209_motor_init(void)
{
    tmc2209_motor_reset_runtime_state();
    return tmc2209_init_backend();
}

void tmc2209_motor_task(void)
{
    if (!s_tmc_ready) return;

    uint32_t now = s_drv.io.get_tick(s_drv.io.ctx);
    if (now - s_diag_last_ms >= DIAG_INTERVAL_MS) {
        tmc2209_drv_status_t st = {0};
        if (tmc2209_get_drv_status(&s_drv, &st) == TMC2209_OK) {
            s_diag_cached = st;
            s_diag_valid  = 1;
        }
        s_diag_last_ms = now;
    }
}

int tmc2209_motor_is_ready(void)
{
    return s_tmc_ready;
}

int tmc2209_motor_set_enabled(int enabled)
{
    if (!s_tmc_ready) return -1;
    tmc2209_result_t res;
    if (enabled) res = tmc2209_enable(&s_drv);
    else         res = tmc2209_disable(&s_drv);
    
    return map_result(res);
}

/* ---- Backend Motion Control ---- */

void tmc2209_motor_stop(void)
{
    if (!s_tmc_ready) return;
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    s_drv.io.motor_stop(s_drv.io.ctx);
    g_pwm_running = 0;
    g_cur_arr     = 0;
#else
    tmc2209_stop(&s_drv);
#endif
}

int tmc2209_motor_is_moving(void)
{
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    return g_pwm_running;
#else
    return s_tmc_ready && (s_drv.vactual != 0);
#endif
}

void tmc2209_motor_move_steps(int32_t steps)
{
    if (!s_tmc_ready) return;
#if MOTOR_DIR_INVERT
    steps = -steps;
#endif

#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    stepdir_steps_internal(steps);
#else
    if (steps == 0) {
        tmc2209_stop(&s_drv);
        return;
    }
    int32_t v = steps * 256;
    if (v > 8388607)  v = 8388607;
    if (v < -8388608) v = -8388608;
    tmc2209_set_vactual(&s_drv, v);
#endif
}

void tmc2209_motor_move_velocity(int32_t velocity)
{
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_UART_VAL
    if (!s_tmc_ready) return;
    if (velocity > 8388607)  velocity = 8388607;
    if (velocity < -8388608) velocity = -8388608;
    tmc2209_set_vactual(&s_drv, velocity);
#else
    (void)velocity;
#endif
}

int tmc2209_motor_set_current(uint16_t run_ma, uint16_t hold_ma)
{
    if (!s_tmc_ready) return -1;
    tmc2209_result_t res = tmc2209_set_current(&s_drv, run_ma, hold_ma);
    if (res == TMC2209_OK) {
        s_run_ma  = run_ma;
        s_hold_ma = hold_ma;
    }
    return map_result(res);
}

int tmc2209_motor_set_microsteps(uint16_t microsteps)
{
    if (!s_tmc_ready) return -1;
    tmc2209_result_t res = tmc2209_set_microsteps(&s_drv, microsteps);
    if (res == TMC2209_OK) {
        s_microsteps = microsteps;
    }
    return map_result(res);
}

void tmc2209_motor_get_config(tmc2209_motor_config_t *cfg)
{
    if (!cfg) return;
    cfg->run_ma     = s_run_ma;
    cfg->hold_ma    = s_hold_ma;
    cfg->microsteps = s_microsteps;
    cfg->mode       = tmc2209_motor_get_control_mode();
    cfg->ready      = s_tmc_ready;
}

tmc2209_motor_mode_t tmc2209_motor_get_control_mode(void)
{
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_UART_VAL
    return TMC2209_MOTOR_CONTROL_UART;
#else
    return TMC2209_MOTOR_CONTROL_STEP_DIR;
#endif
}

int tmc2209_motor_get_version(uint8_t *version)
{
    if (!s_tmc_ready || !version) return -1;
    return (tmc2209_get_version(&s_drv, version) == TMC2209_OK) ? 0 : -1;
}

int tmc2209_motor_get_drv_status(tmc2209_drv_status_t *st)
{
    if (!s_tmc_ready || !st) return -1;
    return (tmc2209_get_drv_status(&s_drv, st) == TMC2209_OK) ? 0 : -1;
}

int tmc2209_motor_get_cached_drv_status(tmc2209_drv_status_t *st)
{
    if (!s_diag_valid || !st) return -1;
    *st = s_diag_cached;
    return 0;
}
