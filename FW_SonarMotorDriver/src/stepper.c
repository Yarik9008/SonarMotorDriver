/* stepper.c — Драйвер STEP/DIR для TMC2209 (TIM4 PWM). */

#include "stepper.h"
#include "board.h"

static TIM_HandleTypeDef htim_step;
static TIM_OC_InitTypeDef sConfigOC = {0};
static uint8_t g_pwm_running = 0;

#define TICKS_PER_US     (TIM4_CLK_HZ / 1000000U)
#define PULSE_TICKS      (STEP_PULSE_US * TICKS_PER_US)
#define MIN_PERIOD_TICKS (PULSE_TICKS + 1U)

void Stepper_Init(void)
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
    Stepper_SetEnable(0);

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

void Stepper_SetEnable(uint8_t enabled)
{
    HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void Stepper_SetDir(uint8_t dir_cw)
{
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, dir_cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Stepper_Stop(void)
{
    if (g_pwm_running) {
        HAL_TIM_PWM_Stop(&htim_step, TIM_CHANNEL_3);
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
        g_pwm_running = 0;
    }
}

void Stepper_Steps(int32_t steps)
{
    Stepper_Stop();

    if (steps == 0)
        return;

    uint32_t n = (uint32_t)(steps < 0 ? -steps : steps);
    uint8_t dir = (steps > 0) ? 1 : 0;
    Stepper_SetDir(dir);

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
