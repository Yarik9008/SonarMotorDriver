/* motor_driver.c — Единый motor-driver слой: TMC2209 + STEP/DIR + UART motion.
 *
 * Единственный владелец ENN. Управление двигателем через TMC2209 library.
 * Backend: STEP_DIR (TIM4 PWM) или UART (VACTUAL).
 * Режим задаётся MOTOR_DRIVER_MODE в board.h.
 */

#include "motor_driver.h"
#include "board.h"
#include "tmc2209_port_stm32_hal.h"
#include "tmc2209/tmc2209.h"

/* ---- TMC2209 internal (blocking init, no state machine) ---- */

static tmc2209_t          s_drv;
static UART_HandleTypeDef  s_huart;
static tmc2209_hal_ctx_t   s_hal;
static uint8_t            s_tmc_ready = 0;

static int tmc2209_init_blocking(void)
{
    s_huart.Instance          = TMC2209_UART;
    s_huart.Init.BaudRate     = TMC2209_UART_BAUDRATE;
    s_huart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_huart.Init.StopBits     = UART_STOPBITS_1;
    s_huart.Init.Parity       = UART_PARITY_NONE;
    s_huart.Init.Mode         = UART_MODE_TX_RX;
    s_huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_huart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_huart);

    s_hal.huart       = &s_huart;
    s_hal.en_port     = ENABLE_PORT;
    s_hal.en_pin      = ENABLE_PIN;
    s_hal.sysclk_hz   = SYSCLK_HZ;
    s_hal.half_duplex = TMC2209_HALF_DUPLEX;
    s_hal.debug_fn    = NULL;

    tmc2209_io_t io;
    tmc2209_port_stm32_hal_fill_io(&io, &s_hal);

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
    return -1;
}

/* ---- STEP/DIR backend (из stepper.c) ---- */

#define TICKS_PER_US     (TIM4_CLK_HZ / 1000000U)
#define PULSE_TICKS      (STEP_PULSE_US * TICKS_PER_US)
#define MIN_PERIOD_TICKS (PULSE_TICKS + 1U)

static TIM_HandleTypeDef htim_step;
static TIM_OC_InitTypeDef sConfigOC = {0};
static uint8_t g_pwm_running = 0;

static void stepdir_set_dir(uint8_t dir_cw)
{
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, dir_cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stepdir_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = STEP_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(STEP_PORT, &gpio);
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);

    gpio.Pin   = DIR_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(DIR_PORT, &gpio);
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);

    gpio.Pin = ENABLE_PIN;
    HAL_GPIO_Init(ENABLE_PORT, &gpio);
    HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, GPIO_PIN_SET);

    htim_step.Instance               = TIM4;
    htim_step.Init.Prescaler         = 0;
    htim_step.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim_step.Init.Period            = 999;
    htim_step.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_step.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim_step);

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.Pulse      = PULSE_TICKS;
    HAL_TIM_PWM_ConfigChannel(&htim_step, &sConfigOC, TIM_CHANNEL_3);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
        __HAL_RCC_TIM4_CLK_ENABLE();
}

static void stepdir_stop(void)
{
    if (g_pwm_running) {
        HAL_TIM_PWM_Stop(&htim_step, TIM_CHANNEL_3);
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
        g_pwm_running = 0;
    }
}

static void stepdir_steps(int32_t steps)
{
    stepdir_stop();

    if (steps == 0)
        return;

    uint32_t n = (uint32_t)(steps < 0 ? -steps : steps);
    uint8_t dir = (steps > 0) ? 1 : 0;
    stepdir_set_dir(dir);

    uint32_t period_us = 1000U / n;
    if (period_us < STEP_PULSE_US + 1U)
        period_us = STEP_PULSE_US + 1U;

    uint32_t arr = period_us * TICKS_PER_US;
    if (arr < MIN_PERIOD_TICKS)
        arr = MIN_PERIOD_TICKS;
    if (arr > 65535U)
        arr = 65535U;

    __HAL_TIM_SET_AUTORELOAD(&htim_step, arr - 1U);
    __HAL_TIM_SET_COMPARE(&htim_step, TIM_CHANNEL_3, (PULSE_TICKS < arr) ? PULSE_TICKS : (arr / 2U));
    __HAL_TIM_SET_COUNTER(&htim_step, 0);
    HAL_TIM_PWM_Start(&htim_step, TIM_CHANNEL_3);
    g_pwm_running = 1;
}

/* ---- Public API ---- */

void MotorDriver_Init(void)
{
    if (tmc2209_init_blocking() != 0)
        return;

#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    stepdir_init();
#endif

    tmc2209_disable(&s_drv);
}

uint8_t MotorDriver_IsReady(void)
{
    return s_tmc_ready;
}

motor_driver_mode_t MotorDriver_GetControlMode(void)
{
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_UART_VAL
    return MOTOR_DRIVER_MODE_UART;  /* enum value */
#else
    return MOTOR_DRIVER_MODE_STEP_DIR;  /* enum value */
#endif
}

void MotorDriver_SetEnabled(uint8_t enabled)
{
    if (!s_tmc_ready) return;
    if (enabled)
        tmc2209_enable(&s_drv);
    else
        tmc2209_disable(&s_drv);
}

int MotorDriver_ConfigureCurrent(uint16_t run_ma, uint16_t hold_ma)
{
    if (!s_tmc_ready) return -1;
    return (tmc2209_set_current(&s_drv, run_ma, hold_ma) == TMC2209_OK) ? 0 : -1;
}

int MotorDriver_ConfigureMicrosteps(uint16_t microsteps)
{
    if (!s_tmc_ready) return -1;
    return (tmc2209_set_microsteps(&s_drv, microsteps) == TMC2209_OK) ? 0 : -1;
}

void MotorDriver_MoveSteps(int32_t steps)
{
#if MOTOR_DIR_INVERT
    steps = -steps;
#endif

#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    stepdir_steps(steps);
#else
    /* UART mode: steps per tick → VACTUAL. VACTUAL scaling: steps/sec ≈ vactual / 256 */
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

void MotorDriver_MoveVelocity(int32_t velocity)
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

void MotorDriver_Stop(void)
{
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    stepdir_stop();
#else
    tmc2209_stop(&s_drv);
#endif
}

int MotorDriver_GetVersion(uint8_t *version)
{
    if (!s_tmc_ready || !version) return -1;
    return (tmc2209_get_version(&s_drv, version) == TMC2209_OK) ? 0 : -1;
}

int MotorDriver_GetDrvStatus(tmc2209_drv_status_t *st)
{
    if (!s_tmc_ready || !st) return -1;
    return (tmc2209_get_drv_status(&s_drv, st) == TMC2209_OK) ? 0 : -1;
}
