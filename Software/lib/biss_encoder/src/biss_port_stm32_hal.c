/**
 * @file biss_port_stm32_hal.c
 * @brief Реализация порта BiSS-C на STM32Cube HAL.
 */

#if defined(USE_HAL_DRIVER)

#include "biss_encoder/biss_port_stm32_hal.h"
#include <string.h>

static biss_hal_ctx_t *s_active_ctx;

static void hal_set_rx_mode(biss_hal_ctx_t *ctx)
{
    HAL_GPIO_WritePin(ctx->de_port, ctx->de_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ctx->re_port, ctx->re_pin, GPIO_PIN_RESET);
}

static int hal_init(void *ctx_ptr)
{
    biss_hal_ctx_t *ctx = (biss_hal_ctx_t *)ctx_ptr;
    if (!ctx)
        return -1;

    s_active_ctx = ctx;

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = ctx->de_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(ctx->de_port, &gpio);

    gpio.Pin = ctx->re_pin;
    HAL_GPIO_Init(ctx->re_port, &gpio);

    hal_set_rx_mode(ctx);

    SPI_HandleTypeDef *hspi = &ctx->hspi;
    hspi->Instance               = SPI1;
    hspi->Init.Mode              = SPI_MODE_MASTER;
    hspi->Init.Direction         = SPI_DIRECTION_2LINES;
    hspi->Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi->Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi->Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi->Init.NSS               = SPI_NSS_SOFT;
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi->Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi->Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;

    if (HAL_SPI_Init(hspi) != HAL_OK)
        return -1;

    return 0;
}

static int hal_spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len,
                        uint32_t timeout_ms, void *ctx_ptr)
{
    biss_hal_ctx_t *ctx = (biss_hal_ctx_t *)ctx_ptr;
    if (!ctx)
        return -1;

    hal_set_rx_mode(ctx);

    if (HAL_SPI_TransmitReceive(&ctx->hspi, (uint8_t *)tx, rx, (uint16_t)len,
                                timeout_ms) != HAL_OK)
        return -1;

    return 0;
}

static int hal_spi_async_start(const uint8_t *tx, uint8_t *rx, size_t len,
                               void *ctx_ptr)
{
    biss_hal_ctx_t *ctx = (biss_hal_ctx_t *)ctx_ptr;
    if (!ctx || !ctx->use_dma)
        return -1;

    ctx->hspi.hdmarx->State = HAL_DMA_STATE_READY;
    ctx->hspi.hdmatx->State = HAL_DMA_STATE_READY;
    ctx->hspi.State         = HAL_SPI_STATE_READY;

    ctx->dma_done  = 0;
    ctx->dma_error = 0;

    hal_set_rx_mode(ctx);

    if (HAL_SPI_TransmitReceive_DMA(&ctx->hspi, (uint8_t *)tx, rx,
                                    (uint16_t)len) != HAL_OK) {
        ctx->dma_error = 1;
        ctx->dma_done  = 1;
        return -1;
    }

    return 0;
}

static uint8_t hal_spi_async_poll(void *ctx_ptr)
{
    biss_hal_ctx_t *ctx = (biss_hal_ctx_t *)ctx_ptr;
    if (!ctx)
        return BISS_ASYNC_ERROR;

    if (!ctx->dma_done)
        return BISS_ASYNC_BUSY;

    return ctx->dma_error ? BISS_ASYNC_ERROR : BISS_ASYNC_OK;
}

static void hal_spi_abort(void *ctx_ptr)
{
    biss_hal_ctx_t *ctx = (biss_hal_ctx_t *)ctx_ptr;
    if (!ctx)
        return;

    SPI_HandleTypeDef *hspi = &ctx->hspi;

    CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    __HAL_DMA_DISABLE(&ctx->hdma_tx);
    __HAL_DMA_DISABLE(&ctx->hdma_rx);
    ctx->hdma_tx.State = HAL_DMA_STATE_READY;
    ctx->hdma_rx.State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(&ctx->hdma_tx);
    __HAL_UNLOCK(&ctx->hdma_rx);

    if (hspi->Instance == SPI1)
        DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;

    __HAL_SPI_CLEAR_OVRFLAG(hspi);
    hspi->State = HAL_SPI_STATE_READY;
    __HAL_UNLOCK(hspi);

    ctx->dma_error = 1;
    ctx->dma_done  = 1;
}

void biss_port_stm32_hal_fill(biss_port_t *port, biss_hal_ctx_t *ctx)
{
    if (!port || !ctx)
        return;

    port->init                  = hal_init;
    port->spi_xfer              = hal_spi_xfer;
    port->spi_xfer_async_start  = ctx->use_dma ? hal_spi_async_start : NULL;
    port->spi_xfer_async_poll   = ctx->use_dma ? hal_spi_async_poll : NULL;
    port->spi_abort             = ctx->use_dma ? hal_spi_abort : NULL;
    port->ctx                   = ctx;
}

void biss_port_stm32_hal_spi_msp_init(SPI_HandleTypeDef *hspi, biss_hal_ctx_t *ctx)
{
    if (!hspi || !ctx || hspi->Instance != SPI1)
        return;

    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = GPIO_PIN_5;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin  = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    ctx->hdma_rx.Instance                 = DMA1_Channel2;
    ctx->hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    ctx->hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    ctx->hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    ctx->hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    ctx->hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    ctx->hdma_rx.Init.Mode                = DMA_NORMAL;
    ctx->hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&ctx->hdma_rx);
    __HAL_LINKDMA(hspi, hdmarx, ctx->hdma_rx);

    ctx->hdma_tx.Instance                 = DMA1_Channel3;
    ctx->hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    ctx->hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    ctx->hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    ctx->hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    ctx->hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    ctx->hdma_tx.Init.Mode                = DMA_NORMAL;
    ctx->hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&ctx->hdma_tx);
    __HAL_LINKDMA(hspi, hdmatx, ctx->hdma_tx);

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, ctx->dma_irq_prio, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, ctx->dma_irq_prio, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

void biss_port_stm32_hal_spi_msp_deinit(SPI_HandleTypeDef *hspi, biss_hal_ctx_t *ctx)
{
    (void)ctx;
    if (!hspi || hspi->Instance != SPI1)
        return;

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6);
    HAL_DMA_DeInit(hspi->hdmarx);
    HAL_DMA_DeInit(hspi->hdmatx);
    __HAL_RCC_SPI1_CLK_DISABLE();
}

void biss_port_stm32_hal_on_spi_complete(SPI_HandleTypeDef *hspi, biss_hal_ctx_t *ctx)
{
    if (!ctx || hspi != &ctx->hspi)
        return;

    ctx->dma_done = 1;
}

void biss_port_stm32_hal_on_spi_error(SPI_HandleTypeDef *hspi, biss_hal_ctx_t *ctx)
{
    if (!ctx || hspi != &ctx->hspi)
        return;

    ctx->dma_error = 1;
    ctx->dma_done  = 1;
}

void biss_port_stm32_hal_dma_rx_irq(biss_hal_ctx_t *ctx)
{
    if (ctx)
        HAL_DMA_IRQHandler(&ctx->hdma_rx);
}

void biss_port_stm32_hal_dma_tx_irq(biss_hal_ctx_t *ctx)
{
    if (ctx)
        HAL_DMA_IRQHandler(&ctx->hdma_tx);
}

biss_hal_ctx_t *biss_port_stm32_hal_get_active_ctx(void)
{
    return s_active_ctx;
}

#endif /* USE_HAL_DRIVER */
