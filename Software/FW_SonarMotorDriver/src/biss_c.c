/**
 * @file biss_c.c
 * @brief Реализация драйвера BiSS-C на базе SPI + DMA.
 *
 * Особенности:
 * - Используется SPI в режиме Master (только тактирование и прием).
 * - RS-485 трансивер управляется через пины DE/RE (обычно DE=1, RE=0 для приема).
 * - Поддерживается программный расчет CRC6 для валидации данных.
 * - Реализованы как блокирующий (для старта), так и неблокирующий (DMA) режимы чтения.
 */

#include "biss_c.h"
#include "board.h"
#include <string.h>

/* --- Состояние драйвера --- */

static SPI_HandleTypeDef hspi_biss;
static DMA_HandleTypeDef hdma_spi_rx;
static DMA_HandleTypeDef hdma_spi_tx;

static uint8_t g_resolution;

static GPIO_TypeDef *g_de_port;
static uint16_t      g_de_pin;
static GPIO_TypeDef *g_re_port;
static uint16_t      g_re_pin;

static uint8_t g_tx_buf[BISS_FRAME_BYTES];
static uint8_t g_rx_buf[BISS_FRAME_BYTES];
static volatile uint8_t g_dma_done = 0;

/* --- CRC --- */

static uint8_t biss_crc6(uint32_t data, uint8_t nbits)
{
    uint8_t crc = 0;
    for (int i = nbits - 1; i >= 0; i--) {
        uint8_t fb = ((crc >> 5) ^ ((data >> i) & 1)) & 1;
        crc = (crc << 1) & 0x3F;
        if (fb)
            crc ^= 0x03;
    }
    return crc ^ 0x3F;
}

/* --- Извлечение бита из буфера --- */

static inline uint8_t rx_bit(const uint8_t *buf, int pos)
{
    return (buf[pos >> 3] >> (7 - (pos & 7))) & 1;
}

/* --- Разбор кадра --- */

static BiSS_Status biss_parse_frame(const uint8_t *rx, BiSS_Reading *out)
{
    memcpy(out->spi_dump, rx, BISS_FRAME_BYTES);

    const int total_bits = BISS_FRAME_BYTES * 8;
    int bp = 0;

    while (bp < total_bits && rx_bit(rx, bp))
        bp++;

    if (bp >= total_bits) {
        out->status = BISS_ERR_NO_RESPONSE;
        return BISS_ERR_NO_RESPONSE;
    }

    while (bp < total_bits && !rx_bit(rx, bp))
        bp++;

    bp += 2;

    if (bp + BISS_SCD_BITS > total_bits) {
        out->status = BISS_ERR_NO_RESPONSE;
        return BISS_ERR_NO_RESPONSE;
    }

    uint32_t scd = 0;
    for (int i = 0; i < (int)BISS_SCD_BITS; i++)
        scd = (scd << 1) | rx_bit(rx, bp + i);

    uint32_t raw_pos = (scd >> 8) & 0x00FFFFFF;
    uint8_t  err_bit = (scd >> 7) & 1;
    uint8_t  wrn_bit = (scd >> 6) & 1;
    uint8_t  crc_rx  = scd & 0x3F;

    uint32_t crc_data = (raw_pos << 2) | (err_bit << 1) | wrn_bit;
    uint8_t  crc_calc = biss_crc6(crc_data, BISS_POSITION_BITS + 2);

    out->raw_position = raw_pos;
    out->position     = raw_pos >> (BISS_POSITION_BITS - g_resolution);
    out->angle_deg    = (float)out->position / (float)(1u << g_resolution) * 360.0f;
    out->error        = err_bit;
    out->warning      = wrn_bit;

    if (crc_rx != crc_calc) {
        out->status = BISS_ERR_CRC;
        return BISS_ERR_CRC;
    }

    if (!err_bit) {
        out->status = BISS_ERR_SENSOR;
        return BISS_ERR_SENSOR;
    }

    if (!wrn_bit) {
        out->status = BISS_ERR_WARNING;
        return BISS_ERR_WARNING;
    }

    out->status = BISS_OK;
    return BISS_OK;
}

/* --- DMA колбэк --- */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
        g_dma_done = 1;
}

/* --- DMA IRQ --- */

void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi_rx);
}

void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi_tx);
}

/* --- Инициализация --- */

BiSS_Status BiSS_Init(const BiSS_Config *cfg)
{
    g_resolution = cfg->resolution_bits;
    g_de_port    = cfg->de_port;
    g_de_pin     = cfg->de_pin;
    g_re_port    = cfg->re_port;
    g_re_pin     = cfg->re_pin;

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = g_de_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(g_de_port, &gpio);
    HAL_GPIO_WritePin(g_de_port, g_de_pin, GPIO_PIN_SET);

    gpio.Pin = g_re_pin;
    HAL_GPIO_Init(g_re_port, &gpio);
    HAL_GPIO_WritePin(g_re_port, g_re_pin, GPIO_PIN_RESET);

    hspi_biss.Instance               = cfg->spi_instance;
    hspi_biss.Init.Mode              = SPI_MODE_MASTER;
    hspi_biss.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi_biss.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi_biss.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi_biss.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi_biss.Init.NSS               = SPI_NSS_SOFT;
    hspi_biss.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi_biss.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi_biss.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi_biss.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;

    if (HAL_SPI_Init(&hspi_biss) != HAL_OK)
        return BISS_ERR_SPI;

    memset(g_tx_buf, 0xFF, sizeof(g_tx_buf));

    return BISS_OK;
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
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

        /* DMA1_Channel2 = SPI1_RX */
        hdma_spi_rx.Instance                 = DMA1_Channel2;
        hdma_spi_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_spi_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_spi_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_spi_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_spi_rx.Init.Mode                = DMA_NORMAL;
        hdma_spi_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        HAL_DMA_Init(&hdma_spi_rx);
        __HAL_LINKDMA(hspi, hdmarx, hdma_spi_rx);

        /* DMA1_Channel3 = SPI1_TX */
        hdma_spi_tx.Instance                 = DMA1_Channel3;
        hdma_spi_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_spi_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_spi_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_spi_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_spi_tx.Init.Mode                = DMA_NORMAL;
        hdma_spi_tx.Init.Priority            = DMA_PRIORITY_LOW;
        HAL_DMA_Init(&hdma_spi_tx);
        __HAL_LINKDMA(hspi, hdmatx, hdma_spi_tx);

        HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
        HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    }
}

/* --- Блокирующее чтение --- */

BiSS_Status BiSS_Read(BiSS_Reading *out)
{
    uint8_t tx[BISS_FRAME_BYTES];
    uint8_t rx[BISS_FRAME_BYTES];

    memset(tx, 0xFF, sizeof(tx));
    memset(rx, 0x00, sizeof(rx));

    if (HAL_SPI_TransmitReceive(&hspi_biss, tx, rx, BISS_FRAME_BYTES, 50) != HAL_OK) {
        memcpy(out->spi_dump, rx, BISS_FRAME_BYTES);
        out->status = BISS_ERR_SPI;
        return BISS_ERR_SPI;
    }

    return biss_parse_frame(rx, out);
}

/* --- Неблокирующее чтение (DMA) --- */

void BiSS_StartRead(void)
{
    g_dma_done = 0;
    memset(g_rx_buf, 0x00, sizeof(g_rx_buf));
    HAL_SPI_TransmitReceive_DMA(&hspi_biss, g_tx_buf, g_rx_buf, BISS_FRAME_BYTES);
}

uint8_t BiSS_IsReady(void)
{
    return g_dma_done;
}

BiSS_Status BiSS_GetResult(BiSS_Reading *out)
{
    return biss_parse_frame(g_rx_buf, out);
}

