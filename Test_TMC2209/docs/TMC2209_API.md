# TMC2209 Library — API Reference

## Overview

Platform-agnostic C library for Trinamic TMC2209 stepper motor driver.
Communicates via single-wire or dual-wire UART, providing:

- Register-level read/write access
- Blocking initialization with communication verification
- Motor current and microstep configuration
- VACTUAL velocity control
- Diagnostic register decoding (IOIN, DRV_STATUS, GSTAT, etc.)
- Explicit error model with typed result codes

The library has **zero platform dependencies**. All hardware access
(UART, GPIO, delays) is routed through user-provided callbacks.

## File Structure

```
lib/tmc2209/
├── include/tmc2209/
│   ├── tmc2209.h          ← public API (main include)
│   ├── tmc2209_types.h    ← result codes, config, decoded register structs
│   ├── tmc2209_regs.h     ← register addresses, bitfield macros
│   └── tmc2209_port.h     ← platform I/O callback interface
├── src/
│   └── tmc2209.c          ← core implementation
└── library.json           ← PlatformIO manifest
```

**Not part of the library** (application/platform code):

| File | Purpose |
|------|---------|
| `include/tmc2209_port_stm32_hal.h` | STM32 HAL port header |
| `src/tmc2209_port_stm32_hal.c` | STM32 HAL port implementation |
| `src/main.c` | Example CLI application |
| `include/board.h` | Board-specific pin/clock config |

## Platform Requirements

The library requires the user to provide 5 mandatory callbacks
and 1 optional callback via `tmc2209_io_t`:

| Callback | Signature | Required |
|----------|-----------|----------|
| `uart_tx` | `int (*)(const uint8_t *data, uint16_t len, uint32_t timeout_ms, void *ctx)` | Yes |
| `uart_rx` | `int (*)(uint8_t *data, uint16_t max_len, uint32_t timeout_ms, uint16_t *received, void *ctx)` | Yes |
| `uart_rx_flush` | `void (*)(void *ctx)` | Yes |
| `delay_us` | `void (*)(uint32_t us, void *ctx)` | Yes |
| `set_enable` | `void (*)(uint8_t level, void *ctx)` | Yes |
| `debug_print` | `void (*)(const char *str, void *ctx)` | No (NULL ok) |

All callbacks receive a `void *ctx` pointer that you set in `tmc2209_io_t.ctx`.

### Callback Contracts

**uart_tx**: Send `len` bytes. Return 0 on success, non-zero on error.

**uart_rx**: Receive up to `max_len` bytes within `timeout_ms`.
Write actual byte count to `*received`.
Return 0 if all bytes received, 1 on timeout (partial data), negative on error.

**uart_rx_flush**: Discard any pending RX data and clear UART error flags.

**delay_us**: Blocking microsecond delay.

**set_enable**: Drive the ENN pin. `level=0` → driver enabled (ENN low),
`level=1` → disabled (ENN high).

## Quick Start

```c
#include "tmc2209/tmc2209.h"

/* 1. Implement platform callbacks (or use a provided port) */
/* 2. Fill config */
tmc2209_config_t cfg = TMC2209_DEFAULT_CONFIG;
cfg.addr     = 0;
cfg.rsense   = 0.11f;
cfg.irun_ma  = 800;
cfg.ihold_ma = 400;

/* 3. Fill I/O callbacks */
tmc2209_io_t io = {
    .uart_tx       = my_uart_tx,
    .uart_rx       = my_uart_rx,
    .uart_rx_flush = my_uart_flush,
    .delay_us      = my_delay_us,
    .set_enable    = my_set_enable,
    .debug_print   = NULL,  /* or my_debug_print */
    .ctx           = &my_platform_ctx,
};

/* 4. Init driver */
tmc2209_t drv;
tmc2209_result_t res = tmc2209_init(&drv, &cfg, &io);
if (res != TMC2209_OK) {
    printf("init failed: %s\n", tmc2209_result_str(res));
    return;
}

/* 5. Enable and run */
tmc2209_enable(&drv);
tmc2209_set_vactual(&drv, 5000);

/* 6. Read diagnostics */
uint8_t version;
tmc2209_get_version(&drv, &version);

tmc2209_drv_status_t status;
tmc2209_get_drv_status(&drv, &status);
printf("cs_actual=%u stst=%u\n", status.cs_actual, status.stst);

/* 7. Stop */
tmc2209_stop(&drv);
tmc2209_disable(&drv);
```

## STM32 HAL Port Example

```c
#include "tmc2209/tmc2209.h"
#include "tmc2209_port_stm32_hal.h"

static UART_HandleTypeDef huart;
static tmc2209_hal_ctx_t  hal_ctx;
static tmc2209_t          drv;

/* Init UART hardware first (HAL_UART_Init, GPIO, etc.) */
/* ... */

/* Fill HAL context */
hal_ctx.huart       = &huart;
hal_ctx.en_port     = GPIOB;
hal_ctx.en_pin      = GPIO_PIN_6;
hal_ctx.sysclk_hz   = 168000000;
hal_ctx.half_duplex = 0;
hal_ctx.debug_fn    = NULL;

/* Fill I/O from port */
tmc2209_io_t io;
tmc2209_port_stm32_hal_fill_io(&io, &hal_ctx);

/* Init */
tmc2209_config_t cfg = TMC2209_DEFAULT_CONFIG;
tmc2209_init(&drv, &cfg, &io);
tmc2209_enable(&drv);
```

## Configuration

`tmc2209_config_t` fields:

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `addr` | `uint8_t` | 0 | UART slave address (0..3, set by MS1/MS2 pins) |
| `rsense` | `float` | 0.11 | Current sense resistor, Ohm |
| `irun_ma` | `uint16_t` | 800 | Run current, mA |
| `ihold_ma` | `uint16_t` | 400 | Hold current, mA |
| `microsteps` | `uint16_t` | 16 | Microstep resolution (1..256, power of 2) |
| `reply_delay_us` | `uint16_t` | 500 | Delay after TX before RX, µs |
| `iholddelay` | `uint8_t` | 4 | IHOLD delay (0..15) |
| `senddelay` | `uint8_t` | 4 | SLAVECONF SENDDELAY (0..15) |
| `pwmconf` | `uint32_t` | 0 | PWMCONF value (0 = library default 0xC10D0024) |
| `tpwmthrs` | `uint32_t` | 0 | TPWMTHRS threshold |

Use `TMC2209_DEFAULT_CONFIG` macro for sensible defaults, then override
individual fields as needed.

## API Reference

### Init / Deinit

```c
tmc2209_result_t tmc2209_init(tmc2209_t *drv, const tmc2209_config_t *cfg,
                              const tmc2209_io_t *io);
void             tmc2209_deinit(tmc2209_t *drv);
```

`tmc2209_init` performs:
1. Writes GCONF (UART enable, register microsteps, multistep filter)
2. Writes IHOLD_IRUN (currents from config)
3. Writes TPWMTHRS, SLAVECONF
4. Reads IFCNT — verifies write communication works
5. Reads IOIN — verifies VERSION == 0x21
6. Configures CHOPCONF (microsteps) and PWMCONF
7. Sets VACTUAL = 0

Does **not** enable the driver — call `tmc2209_enable()` explicitly.

### Low-level Register Access

```c
tmc2209_result_t tmc2209_read_reg(tmc2209_t *drv, uint8_t reg, uint32_t *value);
tmc2209_result_t tmc2209_write_reg(tmc2209_t *drv, uint8_t reg, uint32_t value);
tmc2209_result_t tmc2209_read_reg_addr(tmc2209_t *drv, uint8_t addr,
                                       uint8_t reg, uint32_t *value);
```

Register addresses are defined in `tmc2209_regs.h` (e.g. `TMC2209_REG_DRV_STATUS`).

`tmc2209_read_reg_addr` reads at an arbitrary slave address — useful for
scanning addresses 0..3 to find the chip.

### Configuration

```c
tmc2209_result_t tmc2209_set_current(tmc2209_t *drv, uint16_t run_ma, uint16_t hold_ma);
tmc2209_result_t tmc2209_set_microsteps(tmc2209_t *drv, uint16_t ms);
tmc2209_result_t tmc2209_set_chopconf(tmc2209_t *drv, uint32_t value);
tmc2209_result_t tmc2209_set_pwmconf(tmc2209_t *drv, uint32_t value);
```

`tmc2209_set_current` converts mA to CS register value using the configured
`rsense` and writes IHOLD_IRUN.

`tmc2209_set_microsteps` performs read-modify-write on CHOPCONF to change
the MRES field without altering other chopper settings.

### Motor Control

```c
tmc2209_result_t tmc2209_enable(tmc2209_t *drv);
tmc2209_result_t tmc2209_disable(tmc2209_t *drv);
tmc2209_result_t tmc2209_set_vactual(tmc2209_t *drv, int32_t velocity);
tmc2209_result_t tmc2209_stop(tmc2209_t *drv);
```

`tmc2209_enable/disable` drive the ENN pin via `set_enable` callback.

`tmc2209_set_vactual` writes the VACTUAL register for internal step generation.
Positive = forward, negative = reverse, 0 = stop.

`tmc2209_stop` is equivalent to `tmc2209_set_vactual(drv, 0)`.

### Diagnostics (Decoded)

```c
tmc2209_result_t tmc2209_get_version(tmc2209_t *drv, uint8_t *version);
tmc2209_result_t tmc2209_get_ifcnt(tmc2209_t *drv, uint8_t *count);
tmc2209_result_t tmc2209_get_ioin(tmc2209_t *drv, tmc2209_ioin_t *ioin);
tmc2209_result_t tmc2209_get_drv_status(tmc2209_t *drv, tmc2209_drv_status_t *status);
tmc2209_result_t tmc2209_get_gstat(tmc2209_t *drv, tmc2209_gstat_t *gstat);
tmc2209_result_t tmc2209_get_sg_result(tmc2209_t *drv, uint16_t *result);
tmc2209_result_t tmc2209_get_tstep(tmc2209_t *drv, uint32_t *tstep);
```

These read the raw register and decode into typed structs.
For raw register access, use `tmc2209_read_reg` directly.

### Utility

```c
tmc2209_result_t tmc2209_last_error(const tmc2209_t *drv);
const char      *tmc2209_result_str(tmc2209_result_t res);
```

## Error Codes

| Code | String | Meaning |
|------|--------|---------|
| `TMC2209_OK` | "OK" | Success |
| `TMC2209_ERR_TIMEOUT` | "TIMEOUT" | UART RX timeout, no bytes received |
| `TMC2209_ERR_UART` | "UART_ERROR" | UART TX/RX callback returned error |
| `TMC2209_ERR_BAD_FRAME` | "BAD_FRAME" | Response received but no valid frame found |
| `TMC2209_ERR_CRC` | "CRC_ERROR" | CRC mismatch in response |
| `TMC2209_ERR_INVALID_ARG` | "INVALID_ARG" | NULL pointer or invalid parameter |
| `TMC2209_ERR_NOT_INIT` | "NOT_INITIALIZED" | Driver not initialized |
| `TMC2209_ERR_UNSUPPORTED` | "UNSUPPORTED" | Feature not supported |
| `TMC2209_ERR_HW` | "HW_ERROR" | Hardware error (wrong VERSION, etc.) |

All API functions return `tmc2209_result_t`. Data is returned through
output pointers. The last transport error is also stored in `drv->last_error`.

## Decoded Register Structs

### tmc2209_ioin_t

| Field | Description |
|-------|-------------|
| `enn` | ENN pin state |
| `ms1`, `ms2` | Address select pins |
| `diag` | DIAG output |
| `pdn_uart` | PDN_UART pin |
| `step`, `dir` | STEP/DIR pin states |
| `spread_en` | SpreadCycle enable |
| `version` | IC version (should be 0x21) |

### tmc2209_drv_status_t

| Field | Description |
|-------|-------------|
| `otpw` | Overtemperature pre-warning |
| `ot` | Overtemperature shutdown |
| `s2ga`, `s2gb` | Short to GND (phase A/B) |
| `s2vsa`, `s2vsb` | Short to VS (phase A/B) |
| `ola`, `olb` | Open load (phase A/B) |
| `t120`..`t157` | Temperature threshold flags |
| `cs_actual` | Actual current scale (0..31) |
| `stealth` | StealthChop active |
| `stst` | Standstill detected |

### tmc2209_gstat_t

| Field | Description |
|-------|-------------|
| `reset` | Driver has been reset |
| `drv_err` | Driver error (overtemperature, short) |
| `uv_cp` | Charge pump undervoltage |

## STEP/DIR Mode

STEP/DIR pulse generation is **not** part of this library.
It belongs to the application layer since it is highly platform-specific
(timers, DMA, interrupt priorities).

The library only handles UART-based control (VACTUAL).
See `src/main.c` for an example of STEP/DIR PWM generation using STM32 TIM4.

## Porting to a New Platform

1. Implement the 5 callbacks in `tmc2209_io_t` for your hardware
2. Fill `tmc2209_config_t` with your motor parameters
3. Call `tmc2209_init()` — done

No `#define` board macros, no global state, no compile-time platform selection.
The entire platform dependency is isolated in the callback struct.
