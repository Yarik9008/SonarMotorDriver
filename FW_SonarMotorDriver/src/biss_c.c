/**
 * @file biss_c.c
 * @brief Реализация драйвера BiSS C для энкодеров LENZ IRS.
 *
 * Связь с энкодером организована через RS-422 трансивер THVD1452:
 *   TX: STM32 SPI1_SCK → THVD1452 D → Y/Z (диф. пара) → энкодер MA+/MA-
 *   RX: энкодер SLO+/SLO- → THVD1452 A/B → R → STM32 SPI1_MISO
 *
 * SPI работает как генератор тактового сигнала MA и приёмник данных SLO.
 * При каждом вызове BiSS_Read() отправляется BISS_FRAME_BYTES (6) байт 0xFF,
 * что генерирует 48 тактов MA. Одновременно принимается ответ энкодера
 * на линии MISO.
 *
 * Структура потока бит на MISO:
 *   [ACK: SLO=1] → [Start: SLO=0] → [CDS: 0..N, затем 1] → [SCD: 32 бита]
 */

#include "biss_c.h"
#include <string.h>

/* ======== Состояние драйвера ============================================= */

static SPI_HandleTypeDef hspi_biss;
static uint8_t           g_resolution;  /**< Разрешение энкодера (17 или 18 бит) */
static GPIO_TypeDef     *g_de_port;
static uint16_t          g_de_pin;
static GPIO_TypeDef     *g_re_port;
static uint16_t          g_re_pin;

/* ======== CRC ============================================================ */

/**
 * @brief Вычисление CRC-6 по спецификации BiSS.
 *
 * Полином: x⁶ + x¹ + x⁰ (0x43, сокращённо 0x03).
 * Начальное значение регистра: 0.
 * Финализация: XOR 0x3F (инверсия 6 бит).
 *
 * @param data  Данные (MSB first, значащие биты выровнены к младшим разрядам).
 * @param nbits Количество бит для расчёта (26: 24 поз + Error + Warning).
 * @return CRC6 (6 бит), инвертированная — готова к сравнению с кадром.
 */
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

/* ======== Извлечение бита из буфера ====================================== */

/** Извлечение бита @p pos из массива @p buf (MSB first в каждом байте). */
static inline uint8_t rx_bit(const uint8_t *buf, int pos)
{
    return (buf[pos >> 3] >> (7 - (pos & 7))) & 1;
}

/* ======== Инициализация ================================================== */

/**
 * @brief Инициализация SPI1, GPIO и трансивера THVD1452.
 *
 * Конфигурация SPI1:
 *   - PA5 (SCK)  — AF push-pull, тактовый сигнал MA.
 *   - PA6 (MISO) — вход с подтяжкой к +3.3 В (idle SLO = 1).
 *   - Master, CPOL=0 CPHA=0, 0.75 МГц (APB2 48 МГц / 64).
 *
 * Трансивер THVD1452:
 *   - DE = HIGH → передатчик включён (MA → шина Y/Z).
 *   - RE = LOW  → приёмник включён (шина A/B → SLO).
 *
 * @note Тактирование GPIO-портов для DE/RE должно быть включено
 *       до вызова этой функции (см. BSP_Init в main.c).
 */
BiSS_Status BiSS_Init(const BiSS_Config *cfg)
{
    g_resolution = cfg->resolution_bits;
    g_de_port    = cfg->de_port;
    g_de_pin     = cfg->de_pin;
    g_re_port    = cfg->re_port;
    g_re_pin     = cfg->re_pin;

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = g_de_pin;              /* DE: active HIGH */
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(g_de_port, &gpio);
    HAL_GPIO_WritePin(g_de_port, g_de_pin, GPIO_PIN_SET);

    gpio.Pin = g_re_pin;                /* RE: active LOW */
    HAL_GPIO_Init(g_re_port, &gpio);
    HAL_GPIO_WritePin(g_re_port, g_re_pin, GPIO_PIN_RESET);

    hspi_biss.Instance               = cfg->spi_instance;
    hspi_biss.Init.Mode              = SPI_MODE_MASTER;
    hspi_biss.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi_biss.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi_biss.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi_biss.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi_biss.Init.NSS               = SPI_NSS_SOFT;
    hspi_biss.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;  /* 48 МГц / 64 = 0.75 МГц */
    hspi_biss.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi_biss.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi_biss.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;

    if (HAL_SPI_Init(&hspi_biss) != HAL_OK)
        return BISS_ERR_SPI;

    __HAL_SPI_ENABLE(&hspi_biss);

    return BISS_OK;
}

/**
 * @brief Инициализация аппаратных ресурсов SPI (вызывается из HAL_SPI_Init).
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        __HAL_RCC_SPI1_CLK_ENABLE();
        /* Внимание: тактирование GPIOA включается централизованно в BSP_Init */

        GPIO_InitTypeDef gpio = {0};

        /* PA5 = SPI1_SCK → MA */
        gpio.Pin   = GPIO_PIN_5;
        gpio.Mode  = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &gpio);

        /* PA6 = SPI1_MISO ← SLO */
        gpio.Pin  = GPIO_PIN_6;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}

/* ======== Чтение кадра =================================================== */

/**
 * @brief Чтение одного кадра позиции с энкодера.
 *
 * Алгоритм:
 *   1. SPI отправляет BISS_FRAME_BYTES байт 0xFF → 48 тактов MA.
 *      Одновременно принимается ответ энкодера в rx[].
 *
 *   2. Разбор битового потока:
 *      a) ACK-фаза:   биты = 1 (энкодер подтверждает связь).
 *      b) Start-фаза: бит = 0 (энкодер начинает обработку).
 *      c) CDS-фаза:   биты = 0 (сенсор вычисляет), затем 1 (данные готовы).
 *      d) SCD (32 бита): [24 Position] [Error] [Warning] [6 CRC].
 *
 *   3. Проверка CRC6 по 26 битам (Position + Error + Warning).
 *   4. Валидация битов Error и Warning.
 *
 * Побитовая диаграмма:
 *   1111 0 1 PPPPPPPPPPPPPPPPPPPPPPPP E W CCCCCC
 *   ╰ACK╯ │ ╰──── 24 бита позиции ─────╯ ╰CRC6╯
 *     Start CDS
 *
 * @note SPI-обмен блокирующий (~64 мкс при 0.75 МГц). Это допустимо,
 *       т.к. время обмена << периода опроса (2 мс).
 */
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

    memcpy(out->spi_dump, rx, BISS_FRAME_BYTES);

    const int total_bits = BISS_FRAME_BYTES * 8;  /* 48 бит */
    int bp = 0;

    /* ACK-фаза: пропускаем единицы */
    while (bp < total_bits && rx_bit(rx, bp))
        bp++;

    if (bp >= total_bits) {
        out->status = BISS_ERR_NO_RESPONSE;
        return BISS_ERR_NO_RESPONSE;
    }

    /* Start + CDS-фаза: пропускаем нули, ищем первую единицу (CDS=1) */
    while (bp < total_bits && !rx_bit(rx, bp))
        bp++;

    /* Пропускаем CDS=1 и разделительный бит перед началом SCD */
    bp += 2;

    if (bp + BISS_SCD_BITS > total_bits) {
        out->status = BISS_ERR_NO_RESPONSE;
        return BISS_ERR_NO_RESPONSE;
    }

    /* Извлекаем 32-битный SCD */
    uint32_t scd = 0;
    for (int i = 0; i < (int)BISS_SCD_BITS; i++)
        scd = (scd << 1) | rx_bit(rx, bp + i);

    /* Разбираем поля SCD */
    uint32_t raw_pos = (scd >> 8) & 0x00FFFFFF;
    uint8_t  err_bit = (scd >> 7) & 1;
    uint8_t  wrn_bit = (scd >> 6) & 1;
    uint8_t  crc_rx  = scd & 0x3F;

    /* Проверяем CRC6 по 26 битам: 24 позиция + Error + Warning */
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

/* ======== Утилиты ======================================================== */

const char *BiSS_StatusStr(BiSS_Status st)
{
    static const char *names[] = {
        [BISS_OK]              = "OK",
        [BISS_ERR_CRC]         = "CRC",
        [BISS_ERR_NO_RESPONSE] = "NO_RESP",
        [BISS_ERR_SENSOR]      = "SENSOR",
        [BISS_ERR_WARNING]     = "WARN",
        [BISS_ERR_SPI]         = "SPI",
    };

    if (st < BISS_STATUS_COUNT)
        return names[st];
    return "?";
}
