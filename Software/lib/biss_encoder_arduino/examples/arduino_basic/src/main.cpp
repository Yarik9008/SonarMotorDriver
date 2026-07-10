/**
 * @file main.cpp
 * @brief Пример чтения LENZ IRS по BiSS-C (Arduino / STM32duino).
 *
 * Плата: STM32F103C8 (Blue Pill).
 * SPI1: PA5/PA6/PA7.
 * RS-485 THVD1452: DE=PB0, RE=PB1.
 */

#include <Arduino.h>
#include <SPI.h>
#include "biss_encoder/biss_encoder.h"
#include "biss_encoder/biss_models.h"
#include "biss_encoder/biss_port_arduino.h"

static biss_arduino_ctx_t g_biss_ard;
static biss_encoder_t     g_enc;

static const char *status_str(biss_status_t st)
{
    switch (st) {
    case BISS_OK:             return "OK";
    case BISS_ERR_CRC:          return "CRC";
    case BISS_ERR_NO_RESPONSE:  return "NO_RESP";
    case BISS_ERR_SENSOR:       return "SENSOR";
    case BISS_ERR_WARNING:      return "WARN";
    case BISS_ERR_SPI:          return "SPI";
    default:                    return "?";
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }

    g_biss_ard.spi           = &SPI;
    g_biss_ard.de_pin        = PB0;
    g_biss_ard.re_pin        = PB1;
    g_biss_ard.spi_clock_hz  = 750000;

    biss_port_t port;
    biss_port_arduino_fill(&port, &g_biss_ard);

    biss_encoder_cfg_t cfg = {
        .port  = port,
        .frame = &BISS_LENZ_IRS_17BIT,
    };

    if (biss_encoder_init(&g_enc, &cfg) != BISS_OK) {
        Serial.println("biss_encoder_init failed");
        while (1) { }
    }

    Serial.println("BiSS-C example (Arduino) ready");
}

void loop()
{
    biss_reading_t rd;
    biss_status_t  st = biss_encoder_read(&g_enc, &rd);

    Serial.print("st=");
    Serial.print(status_str(st));
    Serial.print(" pos=");
    Serial.print(rd.position);
    Serial.print(" angle=");
    Serial.print(rd.angle_deg, 2);
    Serial.print(" raw=0x");
    Serial.println(rd.raw_position, HEX);

    delay(100);
}
