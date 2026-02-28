/**
 * @file biss_c.h
 * @brief Драйвер интерфейса BiSS C для абсолютных энкодеров LENZ IRS.
 *
 * Протокол BiSS C — полудуплексный синхронный интерфейс:
 *   - Ведущий (STM32) генерирует тактовый сигнал MA.
 *   - Ведомый (энкодер) отвечает данными на линии SLO.
 *
 * Аппаратное подключение через THVD1452 (полнодуплексный RS-422 трансивер):
 *   STM32 PA5 (SPI_SCK)  →  THVD1452 D  →  Y/Z  →  MA+/MA-   (такт)
 *   STM32 PA6 (SPI_MISO) ←  THVD1452 R  ←  A/B  ←  SLO+/SLO- (данные)
 *   STM32 GPIO (DE)       →  THVD1452 DE  (HIGH = передатчик включён)
 *   STM32 GPIO (RE)       →  THVD1452 RE  (LOW  = приёмник включён)
 *
 * Структура кадра BiSS C:
 *   [ACK: SLO=1] → [Start: SLO=0] → [CDS: 0..N×0, затем 1] → [SCD: 32 бита]
 *
 * Формат SCD (Single Cycle Data, 32 бита, MSB first):
 *   Биты [31:8]  — 24-битная абсолютная позиция
 *   Бит  [7]     — Error   (1 = данные валидны, 0 = ошибка сенсора)
 *   Бит  [6]     — Warning (1 = норма, 0 = проблема с зазором/позицией)
 *   Биты [5:0]   — CRC6 (полином x⁶+x¹+x⁰, start=0, результат инвертирован)
 */

#ifndef BISS_C_H
#define BISS_C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* -------- Константы протокола -------- */

/** Размер SPI-обмена в байтах.
 *  48 бит покрывают: ~4 ACK + 1 Start + ~10 CDS(worst) + 32 SCD + 1 запас */
#define BISS_FRAME_BYTES    6U

#define BISS_SCD_BITS       32U     /**< Длина блока данных SCD */
#define BISS_POSITION_BITS  24U     /**< Разрядность поля позиции внутри SCD */
#define BISS_CRC_BITS       6U      /**< Разрядность CRC в SCD */

/* -------- Коды статуса -------- */

typedef enum {
    BISS_OK = 0,            /**< Чтение успешно, CRC совпала */
    BISS_ERR_CRC,           /**< CRC не совпала — помехи на линии или сбой */
    BISS_ERR_NO_RESPONSE,   /**< Энкодер не ответил (нет ACK → Start → CDS) */
    BISS_ERR_SENSOR,        /**< Бит Error = 0 — данные сенсора невалидны */
    BISS_ERR_WARNING,       /**< Бит Warning = 0 — проблема зазора/позиции */
    BISS_ERR_SPI,           /**< Ошибка HAL SPI при обмене */
    BISS_STATUS_COUNT       /**< Количество статусов (должен быть последним) */
} BiSS_Status;

/* -------- Результат чтения -------- */

typedef struct {
    uint32_t    raw_position;               /**< Сырая 24-битная позиция */
    uint32_t    position;                   /**< Позиция с учётом разрешения (17/18 бит) */
    float       angle_deg;                  /**< Угол в градусах [0; 360) */
    uint8_t     error;                      /**< Бит Error  (1 = ОК) */
    uint8_t     warning;                    /**< Бит Warning (1 = ОК) */
    BiSS_Status status;                     /**< Итоговый статус чтения */
    uint8_t     spi_dump[BISS_FRAME_BYTES]; /**< Сырые байты MISO (диагностика) */
} BiSS_Reading;

/* -------- Конфигурация -------- */

/** Параметры инициализации драйвера.
 *  Вызывающий код должен заранее включить тактирование GPIO-портов,
 *  используемых для DE и RE (через __HAL_RCC_GPIOx_CLK_ENABLE). */
typedef struct {
    SPI_TypeDef    *spi_instance;       /**< Экземпляр SPI (SPI1) */
    uint8_t         resolution_bits;    /**< Разрешение: 17 (I34/I50/I60) или 18 (I70/I80/I90) */
    GPIO_TypeDef   *de_port;            /**< Порт пина DE (Driver Enable, active HIGH) */
    uint16_t        de_pin;             /**< Пин DE */
    GPIO_TypeDef   *re_port;            /**< Порт пина RE (Receiver Enable, active LOW) */
    uint16_t        re_pin;             /**< Пин RE */
} BiSS_Config;

/* -------- API -------- */

/**
 * @brief Инициализация SPI и GPIO для связи с энкодером.
 * @param cfg Указатель на структуру конфигурации (не NULL).
 * @return BISS_OK при успехе, BISS_ERR_SPI при ошибке SPI.
 */
BiSS_Status BiSS_Init(const BiSS_Config *cfg);

/**
 * @brief Однократное чтение абсолютной позиции.
 * @param out Указатель на структуру результата (не NULL).
 * @return BISS_OK при успешном чтении, иначе код ошибки.
 */
BiSS_Status BiSS_Read(BiSS_Reading *out);

/**
 * @brief Текстовая мнемоника статуса (для логирования/диагностики).
 * @param st Код статуса.
 * @return Строковый литерал, например "OK", "CRC", "NO_RESP".
 */
const char *BiSS_StatusStr(BiSS_Status st);

#endif /* BISS_C_H */
