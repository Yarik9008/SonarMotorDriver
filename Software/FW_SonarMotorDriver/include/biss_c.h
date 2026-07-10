/**
 * @file biss_c.h
 * @brief Драйвер протокола BiSS-C для энкодеров LENZ IRS.
 *
 * Модуль обеспечивает чтение данных с абсолютных энкодеров по интерфейсу BiSS-C,
 * используя SPI в качестве транспорта и RS-485 трансивер для физического уровня.
 */

#ifndef BISS_C_H
#define BISS_C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define BISS_FRAME_BYTES    6U   /* байт в SPI кадре */
#define BISS_SCD_BITS       32U  /* бит в SCD (single-turn data) */
#define BISS_POSITION_BITS  24U  /* бит позиции в SCD */
#define BISS_CRC_BITS       6U   /* бит CRC */

/* Результат чтения энкодера */
/**
 * @brief Статус чтения энкодера.
 */
typedef enum {
    BISS_OK = 0,            ///< Успешное чтение
    BISS_ERR_CRC,           ///< Ошибка контрольной суммы кадра
    BISS_ERR_NO_RESPONSE,   ///< Энкодер не обнаружен на линии
    BISS_ERR_SENSOR,        ///< Критическая ошибка датчика (бит Error)
    BISS_ERR_WARNING,       ///< Предупреждение от датчика (бит Warning)
    BISS_ERR_SPI,           ///< Ошибка периферии SPI или DMA
    BISS_STATUS_COUNT
} BiSS_Status;

/* Результат чтения: позиция, угол, статус */
typedef struct {
    uint32_t    raw_position;   /* сырая позиция (24 бит) */
    uint32_t    position;       /* позиция с учётом resolution_bits */
    float       angle_deg;      /* угол в градусах (0..360) */
    uint8_t     error;          /* флаг error из SCD */
    uint8_t     warning;        /* флаг warning из SCD */
    BiSS_Status status;         /* итоговый статус */
    uint8_t     spi_dump[BISS_FRAME_BYTES];  /* сырой SPI кадр */
} BiSS_Reading;

/* Конфигурация: SPI, разрешение, пины DE/RE трансивера */
typedef struct {
    SPI_TypeDef    *spi_instance;
    uint8_t         resolution_bits;   /* 17 для IRS-I34/I50/I60 */
    GPIO_TypeDef   *de_port;
    uint16_t        de_pin;
    GPIO_TypeDef   *re_port;
    uint16_t        re_pin;
} BiSS_Config;

/* Инициализация SPI и DMA. */
BiSS_Status BiSS_Init(const BiSS_Config *cfg);

/* Блокирующее чтение (для инициализации). */
BiSS_Status BiSS_Read(BiSS_Reading *out);

/* Запуск неблокирующего чтения через DMA. 0 = запущено, 1 = ошибка SPI/DMA. */
uint8_t BiSS_StartRead(void);

/* Готов ли результат DMA-чтения (включая завершение с ошибкой)? */
uint8_t BiSS_IsReady(void);

/* Получить результат последнего чтения. Вызывать после BiSS_IsReady()==1. */
BiSS_Status BiSS_GetResult(BiSS_Reading *out);

/* Аварийное прерывание зависшего DMA-обмена (вызывать при таймауте). */
void BiSS_Abort(void);

#endif /* BISS_C_H */
