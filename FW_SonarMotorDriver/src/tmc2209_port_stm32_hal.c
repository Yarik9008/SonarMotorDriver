/* tmc2209_port_stm32_hal.c — STM32 HAL platform adapter for TMC2209 library.
 *
 * Implements tmc2209_io_t callbacks using:
 *   - HAL_UART_Transmit / HAL_UART_Receive (blocking)
 *   - DWT cycle counter for µs delays
 *   - HAL_GPIO for enable pin
 *
 * Supports both full-duplex and half-duplex UART via hal_ctx.half_duplex flag.
 */

#include "tmc2209_port_stm32_hal.h"

/* ---- DWT µs delay ---- */

static void port_delay_us(uint32_t us, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    uint32_t cycles = us * (hal->sysclk_hz / 1000000U);
    uint32_t t0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - t0) < cycles) { }
}

/* ---- UART ---- */

static int port_uart_tx(const uint8_t *data, uint16_t len,
                        uint32_t timeout_ms, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    if (hal->half_duplex)
        HAL_HalfDuplex_EnableTransmitter(hal->huart);

    HAL_StatusTypeDef st = HAL_UART_Transmit(hal->huart,
                                             (uint8_t *)data, len, timeout_ms);
    if (hal->half_duplex)
        HAL_HalfDuplex_EnableReceiver(hal->huart);

    return (st == HAL_OK) ? 0 : -1;
}

static int port_uart_rx(uint8_t *data, uint16_t max_len,
                        uint32_t timeout_ms, uint16_t *received, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    *received = 0;

    if (max_len == 0) return 0;

    HAL_StatusTypeDef st = HAL_UART_Receive(hal->huart, &data[0], 1, timeout_ms);
    if (st == HAL_TIMEOUT)
        return 1;
    if (st != HAL_OK)
        return -1;
    *received = 1;

    for (uint16_t i = 1; i < max_len; i++) {
        st = HAL_UART_Receive(hal->huart, &data[i], 1, 2);
        if (st != HAL_OK)
            break;
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
    while (__HAL_UART_GET_FLAG(h, UART_FLAG_RXNE))
        (void)h->Instance->DR;
}

/* ---- GPIO ---- */

static void port_set_enable(uint8_t level, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    HAL_GPIO_WritePin(hal->en_port, hal->en_pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ---- Debug ---- */

static void port_debug_print(const char *str, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    if (hal->debug_fn)
        hal->debug_fn(str);
}

/* ---- Fill I/O struct ---- */

void tmc2209_port_stm32_hal_fill_io(tmc2209_io_t *io, tmc2209_hal_ctx_t *hal)
{
    io->uart_tx       = port_uart_tx;
    io->uart_rx       = port_uart_rx;
    io->uart_rx_flush = port_uart_rx_flush;
    io->delay_us      = port_delay_us;
    io->set_enable    = port_set_enable;
    io->debug_print   = hal->debug_fn ? port_debug_print : 0;
    io->ctx           = hal;
}
