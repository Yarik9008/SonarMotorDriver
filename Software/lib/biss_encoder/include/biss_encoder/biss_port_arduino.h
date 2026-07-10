/**
 * @file biss_port_arduino.h
 * @brief Порт BiSS-C для Arduino (STM32duino).
 */

#ifndef BISS_ENCODER_PORT_ARDUINO_H
#define BISS_ENCODER_PORT_ARDUINO_H

#include "biss_encoder/biss_port.h"
#include <stdint.h>

#ifdef __cplusplus
#include <SPI.h>

typedef struct {
    SPIClass *spi;
    uint8_t   de_pin;
    uint8_t   re_pin;
    uint32_t  spi_clock_hz;
} biss_arduino_ctx_t;

void biss_port_arduino_fill(biss_port_t *port, biss_arduino_ctx_t *ctx);

extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* BISS_ENCODER_PORT_ARDUINO_H */
