/**
 * @file biss_protocol.c
 * @brief Реализация парсера BiSS-C.
 */

#include "biss_encoder/biss_protocol.h"
#include <string.h>

uint8_t biss_crc6(uint32_t data, uint8_t nbits)
{
    uint8_t crc = 0;
    for (int i = (int)nbits - 1; i >= 0; i--) {
        uint8_t fb = ((crc >> 5) ^ ((data >> i) & 1U)) & 1U;
        crc = (uint8_t)((crc << 1) & 0x3FU);
        if (fb)
            crc ^= 0x03U;
    }
    return (uint8_t)(crc ^ 0x3FU);
}

static inline uint8_t rx_bit(const uint8_t *buf, int pos)
{
    return (uint8_t)((buf[pos >> 3] >> (7 - (pos & 7))) & 1U);
}

biss_status_t biss_parse_frame(const uint8_t *rx,
                               const biss_frame_cfg_t *cfg,
                               biss_reading_t *out)
{
    if (!rx || !cfg || !out)
        return BISS_ERR_SPI;

    memset(out->spi_dump, 0, sizeof(out->spi_dump));
    memcpy(out->spi_dump, rx, cfg->frame_bytes);

    const int total_bits = (int)cfg->frame_bytes * 8;
    int bp = 0;

    while (bp < total_bits && rx_bit(rx, bp))
        bp++;

    if (bp >= total_bits) {
        out->status = BISS_ERR_NO_RESPONSE;
        return BISS_ERR_NO_RESPONSE;
    }

    while (bp < total_bits && !rx_bit(rx, bp))
        bp++;

    bp += 2;

    if (bp + (int)cfg->scd_bits > total_bits) {
        out->status = BISS_ERR_NO_RESPONSE;
        return BISS_ERR_NO_RESPONSE;
    }

    uint32_t scd = 0;
    for (int i = 0; i < (int)cfg->scd_bits; i++)
        scd = (scd << 1) | rx_bit(rx, bp + i);

    uint32_t raw_pos = (scd >> 8) & 0x00FFFFFFU;
    uint8_t  err_bit = (uint8_t)((scd >> 7) & 1U);
    uint8_t  wrn_bit = (uint8_t)((scd >> 6) & 1U);
    uint8_t  crc_rx  = (uint8_t)(scd & 0x3FU);

    uint32_t crc_data = (raw_pos << 2) | ((uint32_t)err_bit << 1) | wrn_bit;
    uint8_t  crc_calc = biss_crc6(crc_data, (uint8_t)(cfg->position_bits + 2U));

    out->raw_position = raw_pos;
    out->position     = raw_pos >> (cfg->position_bits - cfg->resolution_bits);
    out->angle_deg    = (float)out->position /
                        (float)(1U << cfg->resolution_bits) * 360.0f;
    out->error        = err_bit;
    out->warning      = wrn_bit;

    if (crc_rx != crc_calc) {
        out->status = BISS_ERR_CRC;
        return BISS_ERR_CRC;
    }

    if (cfg->error_ok_high) {
        if (!err_bit) {
            out->status = BISS_ERR_SENSOR;
            return BISS_ERR_SENSOR;
        }
    } else if (err_bit) {
        out->status = BISS_ERR_SENSOR;
        return BISS_ERR_SENSOR;
    }

    if (cfg->warning_ok_high) {
        if (!wrn_bit) {
            out->status = BISS_ERR_WARNING;
            return BISS_ERR_WARNING;
        }
    } else if (wrn_bit) {
        out->status = BISS_ERR_WARNING;
        return BISS_ERR_WARNING;
    }

    out->status = BISS_OK;
    return BISS_OK;
}
