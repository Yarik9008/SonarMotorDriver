/**
 * @file biss_c.h
 * @brief Драйвер интерфейса BiSS C для абсолютных энкодеров LENZ IRS.
 *
 * Протокол BiSS C использует SPI-подобный обмен: ведущий (STM32) генерирует
 * тактовый сигнал MA, ведомый (энкодер) отвечает данными на линии SLO.
 *
 * Аппаратное подключение (нужны RS422-трансиверы):
 *   STM32 SPI_SCK  --> RS422 драйвер   --> MA+/MA-  (такт для энкодера)
 *   STM32 SPI_MISO <-- RS422 приёмник   <-- SLO+/SLO- (данные от энкодера)
 *
 * Пины по умолчанию (SPI1):
 *   PA5 = SCK  -> MA
 *   PA6 = MISO <- SLO
 *
 * Формат кадра BiSS C SCD (32 бита, передаётся старшим битом вперёд):
 *   Биты [31:8]  — 24-битная абсолютная позиция
 *   Бит  [7]     — Error  (1 = данные валидны, 0 = ошибка)
 *   Бит  [6]     — Warning (1 = норма, 0 = проблема с зазором / позицией)
 *   Биты [5:0]   — CRC6 (полином x^6+x^1+x^0, start=0, биты инвертированы)
 */

#ifndef BISS_C_H
#define BISS_C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/** Коды статуса операции чтения энкодера */
typedef enum {
    BISS_OK = 0,            /**< Чтение успешно, CRC совпала */
    BISS_ERR_CRC,           /**< CRC не совпала — помехи на линии или сбой */
    BISS_ERR_NO_RESPONSE,   /**< Энкодер не ответил (нет перехода ACK -> Start -> CDS) */
    BISS_ERR_SENSOR,        /**< Бит Error = 0 — данные от сенсора невалидны */
    BISS_ERR_WARNING,       /**< Бит Warning = 0 — проблема (зазор, восстановление позиции) */
    BISS_ERR_SPI,           /**< Ошибка HAL SPI при обмене данными */
} BiSS_Status;

/** Результат одного чтения энкодера */
typedef struct {
    uint32_t    raw_position; /**< Сырая 24-битная позиция (все биты, включая незначащие) */
    uint32_t    position;     /**< Позиция, обрезанная до реального разрешения (17 или 18 бит) */
    float       angle_deg;    /**< Угол в градусах [0, 360), вычисленный из position */
    uint8_t     error;        /**< Бит Error из кадра (1 = ОК, 0 = ошибка) */
    uint8_t     warning;      /**< Бит Warning из кадра (1 = ОК, 0 = предупреждение) */
    BiSS_Status status;       /**< Итоговый статус чтения */
} BiSS_Reading;

/** Параметры инициализации драйвера */
typedef struct {
    SPI_TypeDef *spi_instance;    /**< Экземпляр SPI (например SPI1) */
    uint8_t      resolution_bits; /**< Разрешение энкодера: 17 или 18 бит */
} BiSS_Config;

/**
 * @brief Инициализация SPI и GPIO для связи с энкодером.
 * @param cfg Указатель на структуру с параметрами.
 * @return BISS_OK при успехе, BISS_ERR_SPI при ошибке.
 */
BiSS_Status BiSS_Init(const BiSS_Config *cfg);

/**
 * @brief Однократное чтение абсолютной позиции с энкодера.
 * @param out Указатель на структуру, куда будет записан результат.
 * @return BISS_OK при успешном чтении, или код ошибки.
 */
BiSS_Status BiSS_Read(BiSS_Reading *out);

#endif
