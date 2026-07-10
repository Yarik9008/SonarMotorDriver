/**
 * @file biss_encoder.h
 * @brief Публичный API драйвера BiSS-C.
 */

#ifndef BISS_ENCODER_H
#define BISS_ENCODER_H

#include "biss_encoder/biss_port.h"
#include "biss_encoder/biss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    biss_port_t        port;
    biss_frame_cfg_t   frame;
    uint8_t            tx_buf[BISS_FRAME_BYTES_MAX];
    uint8_t            rx_buf[BISS_FRAME_BYTES_MAX];
    volatile uint8_t   async_active;
    volatile uint8_t   async_error;
} biss_encoder_t;

typedef struct {
    biss_port_t              port;
    const biss_frame_cfg_t  *frame;
} biss_encoder_cfg_t;

biss_status_t biss_encoder_init(biss_encoder_t *enc, const biss_encoder_cfg_t *cfg);
biss_status_t biss_encoder_read(biss_encoder_t *enc, biss_reading_t *out);
int  biss_encoder_start_read(biss_encoder_t *enc);
int  biss_encoder_is_ready(const biss_encoder_t *enc);
biss_status_t biss_encoder_get_result(biss_encoder_t *enc, biss_reading_t *out);
void biss_encoder_abort(biss_encoder_t *enc);

#ifdef __cplusplus
}
#endif

#endif /* BISS_ENCODER_H */
