/* biss_c.h — Драйвер BiSS C для энкодеров LENZ IRS (SPI + DMA + THVD1452). */

#ifndef BISS_C_H
#define BISS_C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define BISS_FRAME_BYTES    6U   /* байт в SPI кадре */
#define BISS_SCD_BITS       32U  /* бит в SCD (single-turn data) */
#define BISS_POSITION_BITS  24U  /* бит позиции в SCD */
#define BISS_CRC_BITS       6U   /* бит CRC */

/* Результат чтения энкодера */
typedef enum {
    BISS_OK = 0,         /* успех */
    BISS_ERR_CRC,        /* ошибка CRC */
    BISS_ERR_NO_RESPONSE,/* нет ответа от энкодера */
    BISS_ERR_SENSOR,     /* error bit установлен */
    BISS_ERR_WARNING,    /* warning bit (данные могут быть неточными) */
    BISS_ERR_SPI,        /* ошибка SPI/DMA */
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

/* Запуск неблокирующего чтения через DMA. */
void BiSS_StartRead(void);

/* Готов ли результат DMA-чтения? */
uint8_t BiSS_IsReady(void);

/* Получить результат последнего чтения. Вызывать после BiSS_IsReady()==1. */
BiSS_Status BiSS_GetResult(BiSS_Reading *out);

#endif /* BISS_C_H */
