/* tmc2209_port_stm32_hal.c — HAL-адаптер для библиотеки TMC2209 (STM32).
 *
 * Реализует callbacks tmc2209_io_t:
 *   - HAL_UART_Transmit / HAL_UART_Receive (блокирующие)
 *   - DWT для микрозадержек (мкс)
 *   - HAL_GPIO для пина Enable
 * Поддерживается full-duplex и half-duplex через hal_ctx.half_duplex.
 */

#include "tmc2209_port_stm32_hal.h"
#include "board.h"

/* ---- HAL UART MSP ---- */

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    if (h->Instance == TMC2209_UART) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = TMC2209_UART_TX_PIN; gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(TMC2209_UART_TX_PORT, &gpio);
        gpio.Pin = TMC2209_UART_RX_PIN; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(TMC2209_UART_RX_PORT, &gpio);
        return;
    }

    if (h->Instance == UART_INSTANCE) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = UART_TX_PIN; gpio.Mode = GPIO_MODE_AF_PP; gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(UART_TX_PORT, &gpio);
        gpio.Pin = UART_RX_PIN; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(UART_RX_PORT, &gpio);

        /* DMA RX: DMA1_Channel5 — USART1_RX, circular */
        static DMA_HandleTypeDef hdma_rx;
        hdma_rx.Instance                 = DMA1_Channel5;
        hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
        HAL_DMA_Init(&hdma_rx);
        __HAL_LINKDMA(h, hdmarx, hdma_rx);

        /* DMA TX: DMA1_Channel4 — USART1_TX, normal */
        static DMA_HandleTypeDef hdma_tx;
        hdma_tx.Instance                 = DMA1_Channel4;
        hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_tx.Init.Mode                = DMA_NORMAL;
        hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
        HAL_DMA_Init(&hdma_tx);
        __HAL_LINKDMA(h, hdmatx, hdma_tx);

        HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, IRQ_PRIO_UART, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
        HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, IRQ_PRIO_UART, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
        HAL_NVIC_SetPriority(USART1_IRQn, IRQ_PRIO_UART, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *h)
{
    if (h->Instance == TMC2209_UART) {
        __HAL_RCC_USART2_CLK_DISABLE();
        HAL_GPIO_DeInit(TMC2209_UART_TX_PORT, TMC2209_UART_TX_PIN);
        HAL_GPIO_DeInit(TMC2209_UART_RX_PORT, TMC2209_UART_RX_PIN);
        return;
    }
    if (h->Instance == UART_INSTANCE) {
        if (h->hdmarx) HAL_DMA_DeInit(h->hdmarx);
        if (h->hdmatx) HAL_DMA_DeInit(h->hdmatx);
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(UART_TX_PORT, UART_TX_PIN);
        HAL_GPIO_DeInit(UART_RX_PORT, UART_RX_PIN);
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
    }
}

/* ---- Микрозадержка (DWT, мкс) ---- */

static void port_delay_us(uint32_t us, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    uint32_t cycles = us * (hal->sysclk_hz / 1000000U);
    uint32_t t0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - t0) < cycles) { }
}

/* ---- UART: передача (blocking) ---- */

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

/* Приём: первый байт с timeout, остальные — до 2 мс на байт. */
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

/* Очистка буфера UART RX перед новой транзакцией. */
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

/* ---- GPIO: пин Enable драйвера ---- */

static void port_set_enable(uint8_t level, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    HAL_GPIO_WritePin(hal->en_port, hal->en_pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ---- Отладочный вывод (опционально) ---- */

static void port_debug_print(const char *str, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    if (hal->debug_fn)
        hal->debug_fn(str);
}

/* Заполняет структуру I/O колбэками для lib/tmc2209. hal_ctx должен быть валиден всё время. */
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
