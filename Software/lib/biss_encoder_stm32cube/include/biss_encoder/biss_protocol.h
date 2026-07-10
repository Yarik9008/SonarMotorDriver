/**
 * @file biss_protocol.h
 * @brief Парсер кадра BiSS-C (без зависимостей от HAL/Arduino).
 */

#ifndef BISS_ENCODER_PROTOCOL_H
#define BISS_ENCODER_PROTOCOL_H

#include "biss_encoder/biss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t biss_crc6(uint32_t data, uint8_t nbits);

biss_status_t biss_parse_frame(const uint8_t *rx,
                               const biss_frame_cfg_t *cfg,
                               biss_reading_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BISS_ENCODER_PROTOCOL_H */
