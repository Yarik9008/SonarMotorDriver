/**
 * @file biss_types.h
 * @brief Общие типы данных библиотеки BiSS-C.
 */

#ifndef BISS_ENCODER_TYPES_H
#define BISS_ENCODER_TYPES_H

#include <stdint.h>

#define BISS_FRAME_BYTES_MAX  8U

typedef enum {
    BISS_OK = 0,
    BISS_ERR_CRC,
    BISS_ERR_NO_RESPONSE,
    BISS_ERR_SENSOR,
    BISS_ERR_WARNING,
    BISS_ERR_SPI,
    BISS_ERR_NOT_SUPPORTED,
    BISS_STATUS_COUNT
} biss_status_t;

typedef struct {
    uint8_t frame_bytes;
    uint8_t scd_bits;
    uint8_t position_bits;
    uint8_t resolution_bits;
    uint8_t crc_bits;
    /** 1: бит error=1 означает «датчик OK» (LENZ IRS). */
    uint8_t error_ok_high;
    /** 1: бит warning=1 означает «нет предупреждения» (LENZ IRS). */
    uint8_t warning_ok_high;
} biss_frame_cfg_t;

typedef struct {
    uint32_t    raw_position;
    uint32_t    position;
    float       angle_deg;
    uint8_t     error;
    uint8_t     warning;
    biss_status_t status;
    uint8_t     spi_dump[BISS_FRAME_BYTES_MAX];
} biss_reading_t;

#endif /* BISS_ENCODER_TYPES_H */
