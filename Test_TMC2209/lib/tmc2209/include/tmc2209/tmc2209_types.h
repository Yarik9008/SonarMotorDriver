/* tmc2209_types.h — TMC2209 library types, enums, config structures. */

#ifndef TMC2209_TYPES_H
#define TMC2209_TYPES_H

#include <stdint.h>

/* ---- Result codes ---- */

typedef enum {
    TMC2209_OK = 0,
    TMC2209_ERR_TIMEOUT,
    TMC2209_ERR_UART,
    TMC2209_ERR_BAD_FRAME,
    TMC2209_ERR_CRC,
    TMC2209_ERR_INVALID_ARG,
    TMC2209_ERR_NOT_INIT,
    TMC2209_ERR_UNSUPPORTED,
    TMC2209_ERR_HW,
} tmc2209_result_t;

/* ---- Driver configuration ---- */

typedef struct {
    uint8_t  addr;              /* Slave address 0..3 (set by MS1/MS2 pins) */
    float    rsense;            /* Sense resistor value, Ohm (e.g. 0.11) */
    uint16_t irun_ma;           /* Run current, mA */
    uint16_t ihold_ma;          /* Hold current, mA */
    uint16_t microsteps;        /* Microstep resolution: 1,2,4,8,16,32,64,128,256 */
    uint16_t reply_delay_us;    /* Delay between TX request and RX read, µs */
    uint8_t  iholddelay;        /* IHOLDDELAY field (0..15) */
    uint8_t  senddelay;         /* SLAVECONF SENDDELAY (0..15) */
    uint32_t pwmconf;           /* PWMCONF register value (0 = library default 0xC10D0024) */
    uint32_t tpwmthrs;          /* TPWMTHRS threshold (0 = disabled) */
} tmc2209_config_t;

/* Convenience initializer with sensible defaults */
#define TMC2209_DEFAULT_CONFIG { \
    .addr           = 0,         \
    .rsense         = 0.11f,     \
    .irun_ma        = 800,       \
    .ihold_ma       = 400,       \
    .microsteps     = 16,        \
    .reply_delay_us = 500,       \
    .iholddelay     = 4,         \
    .senddelay      = 4,         \
    .pwmconf        = 0,         \
    .tpwmthrs       = 0,         \
}

/* ---- Decoded register structures ---- */

typedef struct {
    uint8_t enn;
    uint8_t ms1;
    uint8_t ms2;
    uint8_t diag;
    uint8_t pdn_uart;
    uint8_t step;
    uint8_t spread_en;
    uint8_t dir;
    uint8_t version;
} tmc2209_ioin_t;

typedef struct {
    uint8_t reset;
    uint8_t drv_err;
    uint8_t uv_cp;
} tmc2209_gstat_t;

typedef struct {
    uint8_t otpw;
    uint8_t ot;
    uint8_t s2ga;
    uint8_t s2gb;
    uint8_t s2vsa;
    uint8_t s2vsb;
    uint8_t ola;
    uint8_t olb;
    uint8_t t120;
    uint8_t t143;
    uint8_t t150;
    uint8_t t157;
    uint8_t cs_actual;
    uint8_t stealth;
    uint8_t stst;
} tmc2209_drv_status_t;

#endif /* TMC2209_TYPES_H */
