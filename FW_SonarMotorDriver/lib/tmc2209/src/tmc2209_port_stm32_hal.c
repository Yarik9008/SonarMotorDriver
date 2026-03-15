/* tmc2209_port_stm32_hal.c — STM32 HAL implementations for TMC2209 port. */

#include "tmc2209/tmc2209_port_stm32_hal.h"
#include "board.h"
#include <stdint.h>

/* ---- Microdelay (DWT) ---- */

static void port_delay_us(uint32_t us, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    uint32_t cycles = us * (hal->sysclk_hz / 1000000U);
    /* DWT must be initialized elsewhere (e.g. board_init) */
    uint32_t t0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - t0) < cycles) { }
}

/* ---- UART ---- */

static int port_uart_tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    if (hal->half_duplex) HAL_HalfDuplex_EnableTransmitter(hal->huart);
    HAL_StatusTypeDef st = HAL_UART_Transmit(hal->huart, (uint8_t *)data, len, timeout_ms);
    if (hal->half_duplex) HAL_HalfDuplex_EnableReceiver(hal->huart);
    return (st == HAL_OK) ? 0 : -1;
}

static int port_uart_rx(uint8_t *data, uint16_t max_len, uint32_t timeout_ms, uint16_t *received, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    *received = 0;
    if (max_len == 0) return 0;

    HAL_StatusTypeDef st = HAL_UART_Receive(hal->huart, &data[0], 1, timeout_ms);
    if (st == HAL_TIMEOUT) return 1;
    if (st != HAL_OK) return -1;
    *received = 1;

    for (uint16_t i = 1; i < max_len; i++) {
        st = HAL_UART_Receive(hal->huart, &data[i], 1, 2);
        if (st != HAL_OK) break;
        (*received)++;
    }
    return (*received == max_len) ? 0 : 1;
}

static void port_uart_rx_flush(void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    UART_HandleTypeDef *h = hal->huart;
    __HAL_UART_CLEAR_OREFLAG(h);
    __HAL_UART_CLEAR_PEFLAG(h);
    __HAL_UART_CLEAR_NEFLAG(h);
    __HAL_UART_CLEAR_FEFLAG(h);
    while (__HAL_UART_GET_FLAG(h, UART_FLAG_RXNE)) (void)h->Instance->DR;
}

/* ---- GPIO & Timer ---- */

static void port_set_enable(uint8_t level, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    HAL_GPIO_WritePin(hal->en_port, hal->en_pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int port_motor_hw_init(void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;

    /* GPIO Pins */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = hal->step_pin;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(hal->step_port, &gpio);
    HAL_GPIO_WritePin(hal->step_port, hal->step_pin, GPIO_PIN_RESET);

    gpio.Pin   = hal->dir_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(hal->dir_port, &gpio);
    HAL_GPIO_WritePin(hal->dir_port, hal->dir_pin, GPIO_PIN_RESET);

    /* Timer initialization is partially expected to be done via hal->htim_step init 
       but we ensure PWM config here if needed, or just rely on it being ready. */
    if (HAL_TIM_PWM_Init(hal->htim_step) != HAL_OK) return -1;

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse        = (hal->htim_step->Init.Period + 1) / 2; /* Default 50% if period set */
    
    if (HAL_TIM_PWM_ConfigChannel(hal->htim_step, &sConfigOC, hal->tim_channel) != HAL_OK) return -1;

    return 0;
}

static void port_motor_set_dir(int8_t dir_cw, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    HAL_GPIO_WritePin(hal->dir_port, hal->dir_pin, dir_cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void port_motor_set_rate(uint16_t arr, uint16_t ccr, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    __HAL_TIM_SET_AUTORELOAD(hal->htim_step, arr - 1U);
    __HAL_TIM_SET_COMPARE(hal->htim_step, hal->tim_channel, ccr);
    
    /* If timer not running, start it */
    if (!(hal->htim_step->Instance->CR1 & TIM_CR1_CEN)) {
        __HAL_TIM_SET_COUNTER(hal->htim_step, 0);
        HAL_TIM_PWM_Start(hal->htim_step, hal->tim_channel);
    }
}

static void port_motor_stop(void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    HAL_TIM_PWM_Stop(hal->htim_step, hal->tim_channel);
    HAL_GPIO_WritePin(hal->step_port, hal->step_pin, GPIO_PIN_RESET);
}

static uint32_t port_get_tick(void *ctx)
{
    (void)ctx;
    return HAL_GetTick();
}

static void port_debug_print(const char *str, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    if (hal->debug_fn) hal->debug_fn(str);
}

void tmc2209_port_stm32_hal_fill_io(tmc2209_io_t *io, tmc2209_hal_ctx_t *hal)
{
    io->uart_tx       = port_uart_tx;
    io->uart_rx       = port_uart_rx;
    io->uart_rx_flush = port_uart_rx_flush;
    io->delay_us      = port_delay_us;
    io->set_enable    = port_set_enable;
    
    io->motor_hw_init  = port_motor_hw_init;
    io->motor_set_dir  = port_motor_set_dir;
    io->motor_set_rate = port_motor_set_rate;
    io->motor_stop     = port_motor_stop;
    io->get_tick       = port_get_tick;

    io->debug_print   = hal->debug_fn ? port_debug_print : 0;
    io->ctx           = hal;
}
