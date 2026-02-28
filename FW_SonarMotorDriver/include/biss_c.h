/**
 * @file biss_c.h
 * @brief Драйвер интерфейса BiSS C для абсолютных энкодеров LENZ IRS.
 *
 * Поддерживает два режима:
 *   BiSS_Read()       — блокирующий (для первого чтения при инициализации)
 *   BiSS_StartRead()  — неблокирующий (DMA), результат в BiSS_IsReady() + BiSS_GetResult()
 */

#ifndef BISS_C_H
#define BISS_C_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define BISS_FRAME_BYTES    6U
#define BISS_SCD_BITS       32U
#define BISS_POSITION_BITS  24U
#define BISS_CRC_BITS       6U

typedef enum {
    BISS_OK = 0,
    BISS_ERR_CRC,
    BISS_ERR_NO_RESPONSE,
    BISS_ERR_SENSOR,
    BISS_ERR_WARNING,
    BISS_ERR_SPI,
    BISS_STATUS_COUNT
} BiSS_Status;

typedef struct {
    uint32_t    raw_position;
    uint32_t    position;
    float       angle_deg;
    uint8_t     error;
    uint8_t     warning;
    BiSS_Status status;
    uint8_t     spi_dump[BISS_FRAME_BYTES];
} BiSS_Reading;

typedef struct {
    SPI_TypeDef    *spi_instance;
    uint8_t         resolution_bits;
    GPIO_TypeDef   *de_port;
    uint16_t        de_pin;
    GPIO_TypeDef   *re_port;
    uint16_t        re_pin;
} BiSS_Config;

BiSS_Status BiSS_Init(const BiSS_Config *cfg);

/** Блокирующее чтение (для инициализации). */
BiSS_Status BiSS_Read(BiSS_Reading *out);

/** Запуск неблокирующего чтения через DMA. */
void BiSS_StartRead(void);

/** Готов ли результат DMA-чтения? */
uint8_t BiSS_IsReady(void);

/** Забрать результат последнего DMA-чтения. Вызывать после BiSS_IsReady()==1. */
BiSS_Status BiSS_GetResult(BiSS_Reading *out);

const char *BiSS_StatusStr(BiSS_Status st);

#endif /* BISS_C_H */
