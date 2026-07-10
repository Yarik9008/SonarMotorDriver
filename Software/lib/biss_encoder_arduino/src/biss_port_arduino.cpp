/**
 * @file biss_port_arduino.cpp
 * @brief Реализация порта BiSS-C для Arduino SPI API.
 */

#include "biss_encoder/biss_port_arduino.h"
#include <Arduino.h>

static int arduino_init(void *ctx_ptr)
{
    biss_arduino_ctx_t *ctx = (biss_arduino_ctx_t *)ctx_ptr;
    if (!ctx || !ctx->spi)
        return -1;

    pinMode(ctx->de_pin, OUTPUT);
    pinMode(ctx->re_pin, OUTPUT);
    digitalWrite(ctx->de_pin, HIGH);
    digitalWrite(ctx->re_pin, LOW);

    ctx->spi->begin();
    return 0;
}

static int arduino_spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len,
                            uint32_t timeout_ms, void *ctx_ptr)
{
    (void)timeout_ms;
    biss_arduino_ctx_t *ctx = (biss_arduino_ctx_t *)ctx_ptr;
    if (!ctx || !ctx->spi)
        return -1;

    digitalWrite(ctx->de_pin, HIGH);
    digitalWrite(ctx->re_pin, LOW);

    ctx->spi->beginTransaction(SPISettings(ctx->spi_clock_hz, MSBFIRST, SPI_MODE0));

    for (size_t i = 0; i < len; i++)
        rx[i] = (uint8_t)ctx->spi->transfer(tx[i]);

    ctx->spi->endTransaction();
    return 0;
}

void biss_port_arduino_fill(biss_port_t *port, biss_arduino_ctx_t *ctx)
{
    if (!port || !ctx)
        return;

    port->init                  = arduino_init;
    port->spi_xfer              = arduino_spi_xfer;
    port->spi_xfer_async_start  = NULL;
    port->spi_xfer_async_poll   = NULL;
    port->spi_abort             = NULL;
    port->ctx                   = ctx;
}
