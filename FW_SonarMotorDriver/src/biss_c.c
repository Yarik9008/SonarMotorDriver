/**
 * @file biss_c.c
 * @brief Реализация драйвера BiSS C для энкодеров LENZ IRS.
 *
 * Связь с энкодером организована через full-duplex RS-422 трансивер THVD1452:
 *   TX: STM32 SPI_SCK → THVD1452 D → Y/Z (диф. пара) → энкодер MA+/MA-
 *   RX: энкодер SLO+/SLO- → THVD1452 A/B → R → STM32 SPI_MISO
 *
 * Управление трансивером:
 *   DE = HIGH — драйвер (передатчик) включён, тактовый сигнал MA идёт на шину
 *   RE = LOW  — приёмник включён, данные SLO поступают в STM32
 *
 * SPI используется как генератор тактового сигнала MA и приёмник данных SLO.
 * При каждом вызове BiSS_Read() STM32 отправляет 10 байт (80 тактов) по SPI,
 * одновременно принимая ответ от энкодера на линии MISO.
 *
 * В полученном потоке битов ищется структура кадра BiSS C:
 *   [ACK: SLO=1] -> [Start: SLO=0] -> [CDS=1: данные готовы] -> [SCD: 32 бита данных]
 */

#include "biss_c.h"
#include <string.h>

/**
 * Размер буфера SPI-обмена в байтах.
 * 80 тактов покрывают: 4 ACK + 1 Start + до 10 CDS + 32 SCD + запас.
 */
#define BISS_FRAME_BYTES  10

static SPI_HandleTypeDef hspi_biss;     /**< Хэндл SPI для связи с энкодером */
static uint8_t           g_resolution;  /**< Разрешение энкодера (17 или 18 бит) */
static GPIO_TypeDef     *g_de_port;     /**< Порт пина DE трансивера THVD1452 */
static uint16_t          g_de_pin;      /**< Пин DE трансивера */
static GPIO_TypeDef     *g_re_port;     /**< Порт пина RE трансивера THVD1452 */
static uint16_t          g_re_pin;      /**< Пин RE трансивера */

/**
 * @brief Вычисление CRC-6 по стандарту BiSS.
 *
 * Полином: x^6 + x^1 + x^0  (0x03)
 * Начальное значение: 0
 * Результат: инвертированный (XOR 0x3F)
 *
 * @param data Данные для расчёта (биты упакованы в uint32_t, MSB first).
 * @param nbits Количество значащих бит в data (обычно 26: 24 позиция + E + W).
 * @return Вычисленная CRC6 (6 бит), уже инвертированная — готова для сравнения.
 */
static uint8_t biss_crc6(uint32_t data, uint8_t nbits)
{
    uint8_t crc = 0;
    for (int i = nbits - 1; i >= 0; i--) {
        /* XOR старшего бита CRC с текущим битом данных */
        uint8_t fb = ((crc >> 5) ^ ((data >> i) & 1)) & 1;
        crc = (crc << 1) & 0x3F; /* Сдвиг регистра, маска 6 бит */
        if (fb)
            crc ^= 0x03; /* Полином: биты x^1 и x^0 */
    }
    return crc ^ 0x3F; /* Инверсия результата по спецификации BiSS */
}

/**
 * @brief Инициализация SPI1, GPIO и управления трансивером THVD1452.
 *
 * Настраивает:
 *   - PA5 (SPI1_SCK)  — альтернативный выход push-pull → D (THVD1452) → MA
 *   - PA6 (SPI1_MISO) — вход с подтяжкой к +3.3В ← R (THVD1452) ← SLO
 *   - DE pin — выход push-pull, HIGH → включает драйвер (TX: MA)
 *   - RE pin — выход push-pull, LOW  → включает приёмник (RX: SLO)
 *   - SPI1 в режиме Master, 0.75 МГц (APB2 48 МГц / 64)
 *
 * Режим SPI: CPOL=0, CPHA=1 (захват по заднему фронту) —
 * оптимален для BiSS C, где данные стабильны на заднем фронте MA.
 */
BiSS_Status BiSS_Init(const BiSS_Config *cfg)
{
    g_resolution = cfg->resolution_bits;
    g_de_port    = cfg->de_port;
    g_de_pin     = cfg->de_pin;
    g_re_port    = cfg->re_port;
    g_re_pin     = cfg->re_pin;

    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* PA5 = SPI1_SCK → THVD1452 D → Y/Z → MA+/MA- */
    gpio.Pin   = GPIO_PIN_5;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA6 = SPI1_MISO ← THVD1452 R ← A/B ← SLO+/SLO- (подтяжка: idle = 1) */
    gpio.Pin  = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* DE (Driver Enable): active HIGH — включаем передатчик MA */
    gpio.Pin   = g_de_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(g_de_port, &gpio);
    HAL_GPIO_WritePin(g_de_port, g_de_pin, GPIO_PIN_SET);

    /* RE (Receiver Enable): active LOW — включаем приёмник SLO */
    gpio.Pin = g_re_pin;
    HAL_GPIO_Init(g_re_port, &gpio);
    HAL_GPIO_WritePin(g_re_port, g_re_pin, GPIO_PIN_RESET);

    /* Настройка SPI1 как Master */
    hspi_biss.Instance               = cfg->spi_instance;
    hspi_biss.Init.Mode              = SPI_MODE_MASTER;
    hspi_biss.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi_biss.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi_biss.Init.CLKPolarity       = SPI_POLARITY_LOW;   /* MA idle = LOW */
    hspi_biss.Init.CLKPhase          = SPI_PHASE_2EDGE;    /* Захват на заднем фронте */
    hspi_biss.Init.NSS               = SPI_NSS_SOFT;       /* CS не используется */
    hspi_biss.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; /* 48 МГц / 64 = 0.75 МГц */
    hspi_biss.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi_biss.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi_biss.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;

    if (HAL_SPI_Init(&hspi_biss) != HAL_OK)
        return BISS_ERR_SPI;

    __HAL_SPI_ENABLE(&hspi_biss);

    return BISS_OK;
}

/**
 * @brief Чтение одного кадра позиции с энкодера.
 *
 * Алгоритм:
 *   1. Через SPI отправляем 10 байт 0xFF (генерируем 80 тактов MA),
 *      одновременно принимаем ответ энкодера в rx[].
 *
 *   2. В принятом потоке битов ищем структуру кадра BiSS C:
 *      a) Пропускаем ACK-фазу (SLO = 1, энкодер подтверждает связь).
 *      b) Пропускаем Start + CDS-фазу (SLO = 0, энкодер обрабатывает запрос).
 *      c) Первый бит «1» после серии «0» — это начало данных (CDS = 1).
 *      d) Следующие 32 бита — это SCD (Single Cycle Data):
 *         [24 бита позиции] [Error] [Warning] [6 бит CRC]
 *
 *   3. Проверяем CRC6 по 26 битам (позиция + Error + Warning).
 *
 *   4. Проверяем биты Error и Warning.
 *
 * Пример потока битов (побитово):
 *   1111 0 1 PPPPPPPPPPPPPPPPPPPPPPPP E W CCCCCC
 *   └ACK┘ │ │└──── 24 бита позиции ─────┘ └CRC6┘
 *     Start CDS
 */
BiSS_Status BiSS_Read(BiSS_Reading *out)
{
    uint8_t tx[BISS_FRAME_BYTES];
    uint8_t rx[BISS_FRAME_BYTES];

    /* TX = 0xFF — при передаче единиц линия MA генерирует такты,
     * а на MISO одновременно принимаются данные от энкодера */
    memset(tx, 0xFF, sizeof(tx));
    memset(rx, 0x00, sizeof(rx));

    /* Полнодуплексный обмен: 80 тактов MA, таймаут 50 мс */
    if (HAL_SPI_TransmitReceive(&hspi_biss, tx, rx, BISS_FRAME_BYTES, 50) != HAL_OK) {
        out->status = BISS_ERR_SPI;
        return BISS_ERR_SPI;
    }

    const int total_bits = BISS_FRAME_BYTES * 8; /* 80 бит */

    /* Макрос для извлечения одного бита из массива rx[] по номеру позиции.
     * pos>>3 — номер байта, 7-(pos&7) — номер бита внутри байта (MSB first) */
#define RX_BIT(pos) ((rx[(pos) >> 3] >> (7 - ((pos) & 7))) & 1)

    /* --- Шаг 1: Пропускаем ACK-фазу (биты = 1) --- */
    int bp = 0;
    while (bp < total_bits && RX_BIT(bp))
        bp++;

    if (bp >= total_bits) {
        /* Все 80 бит — единицы. Энкодер не перешёл к Start-фазе.
         * Нет связи с энкодером или он не подключён. */
        out->status = BISS_ERR_NO_RESPONSE;
        return BISS_ERR_NO_RESPONSE;
    }

    /* --- Шаг 2: Пропускаем Start + CDS-фазу (биты = 0) --- */
    while (bp < total_bits && !RX_BIT(bp))
        bp++;

    if (bp + 32 >= total_bits) {
        /* Недостаточно бит для полного 32-битного SCD */
        out->status = BISS_ERR_NO_RESPONSE;
        return BISS_ERR_NO_RESPONSE;
    }

    /* --- Шаг 3: Извлекаем 32-битный SCD (начиная с бита CDS=1) --- */
    uint32_t scd = 0;
    for (int i = 0; i < 32; i++)
        scd = (scd << 1) | RX_BIT(bp + i);

#undef RX_BIT

    /* --- Шаг 4: Разбираем поля SCD --- */
    uint32_t raw_pos = (scd >> 8) & 0x00FFFFFF; /* Биты [31:8] — 24-бит позиция */
    uint8_t  err_bit = (scd >> 7) & 1;          /* Бит [7] — Error */
    uint8_t  wrn_bit = (scd >> 6) & 1;          /* Бит [6] — Warning */
    uint8_t  crc_rx  = scd & 0x3F;              /* Биты [5:0] — CRC6 от энкодера */

    /* --- Шаг 5: Проверяем CRC6 ---
     * CRC считается по 26 битам: 24 бита позиции + Error + Warning */
    uint32_t crc_data = (raw_pos << 2) | (err_bit << 1) | wrn_bit;
    uint8_t  crc_calc = biss_crc6(crc_data, 26);

    /* Заполняем результат */
    out->raw_position = raw_pos;
    /* Обрезаем до реального разрешения: сдвиг вправо убирает (24 - resolution) младших бит */
    out->position     = raw_pos >> (24 - g_resolution);
    /* Пересчёт в градусы: position / 2^resolution * 360 */
    out->angle_deg    = (float)out->position / (float)(1u << g_resolution) * 360.0f;
    out->error        = err_bit;
    out->warning      = wrn_bit;

    /* --- Шаг 6: Валидация результата --- */
    if (crc_rx != crc_calc) {
        out->status = BISS_ERR_CRC;
        return BISS_ERR_CRC;
    }

    if (!err_bit) {
        /* Error=0 означает, что данные энкодера невалидны */
        out->status = BISS_ERR_SENSOR;
        return BISS_ERR_SENSOR;
    }

    if (!wrn_bit) {
        /* Warning=0 означает проблему: слишком большой зазор ротор-статор
         * или невозможность восстановить позицию после перезапуска */
        out->status = BISS_ERR_WARNING;
        return BISS_ERR_WARNING;
    }

    out->status = BISS_OK;
    return BISS_OK;
}
