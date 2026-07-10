/**
 * @file biss_encoder.c
 * @brief Фасад чтения BiSS-C поверх biss_port_t.
 */

#include "biss_encoder/biss_encoder.h"
#include "biss_encoder/biss_protocol.h"
#include <string.h>

biss_status_t biss_encoder_init(biss_encoder_t *enc, const biss_encoder_cfg_t *cfg)
{
    if (!enc || !cfg || !cfg->frame || !cfg->port.init || !cfg->port.spi_xfer)
        return BISS_ERR_SPI;

    memset(enc, 0, sizeof(*enc));
    enc->port  = cfg->port;
    enc->frame = *cfg->frame;

    if (enc->frame.frame_bytes > BISS_FRAME_BYTES_MAX)
        return BISS_ERR_SPI;

    memset(enc->tx_buf, 0xFF, enc->frame.frame_bytes);

    if (enc->port.init(enc->port.ctx) != 0)
        return BISS_ERR_SPI;

    return BISS_OK;
}

biss_status_t biss_encoder_read(biss_encoder_t *enc, biss_reading_t *out)
{
    if (!enc || !out)
        return BISS_ERR_SPI;

    uint8_t rx[BISS_FRAME_BYTES_MAX];
    memset(rx, 0, sizeof(rx));

    if (enc->port.spi_xfer(enc->tx_buf, rx, enc->frame.frame_bytes, 50U,
                           enc->port.ctx) != 0) {
        memcpy(out->spi_dump, rx, enc->frame.frame_bytes);
        out->status = BISS_ERR_SPI;
        return BISS_ERR_SPI;
    }

    return biss_parse_frame(rx, &enc->frame, out);
}

int biss_encoder_start_read(biss_encoder_t *enc)
{
    if (!enc || !enc->port.spi_xfer_async_start)
        return 1;

    enc->async_active = 1U;
    enc->async_error  = 0U;
    memset(enc->rx_buf, 0, enc->frame.frame_bytes);

    if (enc->port.spi_xfer_async_start(enc->tx_buf, enc->rx_buf,
                                       enc->frame.frame_bytes,
                                       enc->port.ctx) != 0) {
        enc->async_error  = 1U;
        enc->async_active = 0U;
        return 1;
    }

    return 0;
}

int biss_encoder_is_ready(const biss_encoder_t *enc)
{
    if (!enc)
        return 0;

    if (!enc->async_active)
        return enc->async_error ? 1 : 0;

    if (!enc->port.spi_xfer_async_poll)
        return 0;

    uint8_t st = enc->port.spi_xfer_async_poll(enc->port.ctx);
    if (st == BISS_ASYNC_BUSY)
        return 0;

    return 1;
}

biss_status_t biss_encoder_get_result(biss_encoder_t *enc, biss_reading_t *out)
{
    if (!enc || !out)
        return BISS_ERR_SPI;

    if (enc->async_error ||
        (enc->port.spi_xfer_async_poll &&
         enc->port.spi_xfer_async_poll(enc->port.ctx) == BISS_ASYNC_ERROR)) {
        memcpy(out->spi_dump, enc->rx_buf, enc->frame.frame_bytes);
        out->status = BISS_ERR_SPI;
        enc->async_active = 0U;
        return BISS_ERR_SPI;
    }

    enc->async_active = 0U;
    return biss_parse_frame(enc->rx_buf, &enc->frame, out);
}

void biss_encoder_abort(biss_encoder_t *enc)
{
    if (!enc || !enc->port.spi_abort)
        return;

    enc->port.spi_abort(enc->port.ctx);
    enc->async_error  = 1U;
    enc->async_active = 0U;
}
