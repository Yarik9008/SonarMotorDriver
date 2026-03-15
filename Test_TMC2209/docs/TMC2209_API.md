# TMC2209 Library — API Reference

> [!IMPORTANT]
> **Питание драйвера (VS):** Линию питания драйвера VS необходимо **ВСЕГДА** подключать с дополнительным электролитическим конденсатором (100–470 мкФ) для стабильной работы и защиты от пробоя.

## 1. Overview

Platform-agnostic C library for Trinamic TMC2209 stepper motor driver (UART interface).

**Core library** (`lib/tmc2209/`) — zero platform dependencies:
- UART protocol (CRC, framing, read/write)
- Blocking init with communication verification
- Shadow registers for write-only register tracking
- Typed config structs for CHOPCONF, PWMCONF, CoolStep, StallGuard
- Motor current, microsteps, VACTUAL velocity control
- Full diagnostic register decoding
- CoolStep adaptive current control
- StallGuard sensorless stall detection
- OTP memory access
- Multi-device UART support
- StealthChop / SpreadCycle presets

**Platform layer** (application code):
- `tmc2209_port_stm32_hal.c` — STM32 HAL callbacks implementation
- STEP/DIR pulse generation (timers, DMA) — not part of library

**Example application**: `src/main.c` — USB CDC CLI demonstrating library usage.

## 2. Quick Start

```c
#include "tmc2209/tmc2209.h"

tmc2209_config_t cfg = TMC2209_DEFAULT_CONFIG;
cfg.addr     = 0;
cfg.rsense   = 0.11f;
cfg.irun_ma  = 800;
cfg.ihold_ma = 400;

tmc2209_io_t io = {
    .uart_tx       = my_uart_tx,
    .uart_rx       = my_uart_rx,
    .uart_rx_flush = my_uart_flush,
    .delay_us      = my_delay_us,
    .set_enable    = my_set_enable,
    .debug_print   = NULL,
    .ctx           = &my_hw,
};

tmc2209_t drv;
tmc2209_result_t res = tmc2209_init(&drv, &cfg, &io);
if (res != TMC2209_OK) { /* handle error */ }

tmc2209_enable(&drv);
tmc2209_set_vactual(&drv, 5000);
```

## 3. Platform Callbacks

All hardware access is routed through `tmc2209_io_t`:

| Callback | Required | Description |
|----------|----------|-------------|
| `uart_tx` | Yes | Send bytes. Return 0=OK, non-zero=error |
| `uart_rx` | Yes | Receive bytes. Write count to `*received`. Return 0=all, 1=timeout, <0=error |
| `uart_rx_flush` | Yes | Discard pending RX data, clear UART error flags |
| `delay_us` | Yes | Blocking microsecond delay |
| `set_enable` | Yes | Drive ENN pin: 0=enabled (low), 1=disabled (high) |
| `debug_print` | No | Diagnostic string output (NULL to disable) |
| `ctx` | — | Opaque pointer passed to all callbacks |

## 4. Configuration

### tmc2209_config_t

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `addr` | uint8_t | 0 | UART slave address 0..3 (MS1/MS2 pins) |
| `rsense` | float | 0.11 | Sense resistor, Ohm |
| `irun_ma` | uint16_t | 800 | Run current, mA |
| `ihold_ma` | uint16_t | 400 | Hold current, mA |
| `microsteps` | uint16_t | 16 | 1,2,4,8,16,32,64,128,256 |
| `reply_delay_us` | uint16_t | 500 | Delay after TX before RX |
| `iholddelay` | uint8_t | 4 | Hold current ramp delay 0..15 |
| `senddelay` | uint8_t | 4 | UART reply send delay 0..15 |
| `pwmconf` | uint32_t | 0 | PWMCONF raw value (0=library default) |
| `tpwmthrs` | uint32_t | 0 | TPWMTHRS velocity threshold |
| `tpowerdown` | uint8_t | 20 | Power-down delay 0..255 |
| `en_spreadcycle` | uint8_t | 0 | 0=StealthChop, 1=SpreadCycle |

Use `TMC2209_DEFAULT_CONFIG` for sensible defaults.

## 5. API Reference

### Init / Deinit

| Function | Description |
|----------|-------------|
| `tmc2209_init(drv, cfg, io)` | Full blocking init: GCONF, currents, CHOPCONF, PWMCONF, verify IFCNT+VERSION |
| `tmc2209_deinit(drv)` | Stop motor, disable driver, clear state |

### Low-level Register Access

| Function | Description |
|----------|-------------|
| `tmc2209_read_reg(drv, reg, &value)` | Read register at configured address |
| `tmc2209_write_reg(drv, reg, value)` | Write register at configured address |
| `tmc2209_read_reg_addr(drv, addr, reg, &value)` | Read at arbitrary slave address |

### GCONF

| Function | Description |
|----------|-------------|
| `tmc2209_get_gconf(drv, &gc)` | Read GCONF from chip, decode into `tmc2209_gconf_t` |
| `tmc2209_set_shaft(drv, inverted)` | Invert motor direction (0=normal, 1=inverted) |
| `tmc2209_enable_spreadcycle(drv)` | Switch to SpreadCycle chopper mode |
| `tmc2209_enable_stealthchop(drv)` | Switch to StealthChop PWM mode |
| `tmc2209_enable_internal_rsense(drv, en)` | Enable/disable internal sense resistors |

### Current / Power

| Function | Description |
|----------|-------------|
| `tmc2209_set_current(drv, run_ma, hold_ma)` | Set both currents in mA |
| `tmc2209_set_run_current(drv, run_ma)` | Set run current only |
| `tmc2209_set_hold_current(drv, hold_ma)` | Set hold current only |
| `tmc2209_get_current_config(drv, &cc)` | Read CS values from shadow |
| `tmc2209_set_iholddelay(drv, delay)` | Set hold current transition delay 0..15 |
| `tmc2209_set_tpowerdown(drv, value)` | Set power-down delay 0..255 |

### CHOPCONF

| Function | Description |
|----------|-------------|
| `tmc2209_set_chopconf_config(drv, &cc)` | Set from typed `tmc2209_chopconf_t` |
| `tmc2209_get_chopconf_config(drv, &cc)` | Read from chip into typed struct |
| `tmc2209_set_chopconf(drv, value)` | Set from raw uint32_t |
| `tmc2209_set_microsteps(drv, ms)` | Set microstep resolution (validates input) |
| `tmc2209_get_microsteps(drv, &ms)` | Read current resolution from chip |
| `tmc2209_enable_interpolation(drv, en)` | Enable 256-µstep interpolation |
| `tmc2209_enable_double_edge_step(drv, en)` | Enable both-edge STEP counting |

### PWMCONF

| Function | Description |
|----------|-------------|
| `tmc2209_set_pwmconf_config(drv, &pc)` | Set from typed `tmc2209_pwmconf_t` |
| `tmc2209_get_pwmconf_config(drv, &pc)` | Read from shadow (register is write-only) |
| `tmc2209_set_pwmconf(drv, value)` | Set from raw uint32_t |
| `tmc2209_set_freewheel(drv, mode)` | Set standstill freewheel mode |

### CoolStep

| Function | Description |
|----------|-------------|
| `tmc2209_set_coolstep_config(drv, &cs)` | Configure CoolStep from `tmc2209_coolstep_config_t` |
| `tmc2209_get_coolstep_config(drv, &cs)` | Read from shadow (COOLCONF is write-only) |
| `tmc2209_set_tcoolthrs(drv, threshold)` | Set velocity threshold for CoolStep/StallGuard |

### StallGuard

| Function | Description |
|----------|-------------|
| `tmc2209_set_sgthrs(drv, threshold)` | Set StallGuard threshold 0..255 |
| `tmc2209_get_sgthrs(drv, &threshold)` | Read from shadow (SGTHRS is write-only) |
| `tmc2209_configure_stallguard(drv, &sg)` | Set SGTHRS + TCOOLTHRS together |

### Motor Control

| Function | Description |
|----------|-------------|
| `tmc2209_enable(drv)` | Enable driver (ENN low) |
| `tmc2209_disable(drv)` | Disable driver (ENN high) |
| `tmc2209_set_vactual(drv, velocity)` | Set internal step generator velocity |
| `tmc2209_stop(drv)` | Set VACTUAL=0 |

### Standby

| Function | Description |
|----------|-------------|
| `tmc2209_enter_standby(drv)` | Software standby: stop, disable, zero hold, freewheel |
| `tmc2209_exit_standby(drv)` | Restore previous settings and re-enable |

After standby, all register shadows are preserved. If the chip loses power
during standby, call `tmc2209_init()` instead of `tmc2209_exit_standby()`.

### Diagnostics

| Function | Returns |
|----------|---------|
| `tmc2209_get_version(drv, &ver)` | IC version from IOIN (expected 0x21) |
| `tmc2209_get_ifcnt(drv, &cnt)` | UART interface counter |
| `tmc2209_get_ioin(drv, &ioin)` | Pin states → `tmc2209_ioin_t` |
| `tmc2209_get_drv_status(drv, &ds)` | Full status → `tmc2209_drv_status_t` |
| `tmc2209_get_gstat(drv, &gs)` | Global status → `tmc2209_gstat_t` |
| `tmc2209_clear_gstat(drv)` | Clear GSTAT flags |
| `tmc2209_get_sg_result(drv, &sg)` | StallGuard result 0..1023 |
| `tmc2209_get_tstep(drv, &tstep)` | Measured step time |
| `tmc2209_get_cs_actual(drv, &cs)` | Actual current scale 0..31 |
| `tmc2209_get_pwm_scale(drv, &ps)` | PWM scaling → `tmc2209_pwm_scale_t` |
| `tmc2209_get_pwm_auto(drv, &pa)` | Auto-tune results → `tmc2209_pwm_auto_t` |
| `tmc2209_get_mscnt(drv, &cnt)` | Microstep counter 0..1023 |
| `tmc2209_get_mscuract(drv, &mc)` | Phase currents → `tmc2209_mscuract_t` |

### OTP (One-Time Programmable)

> **WARNING**: OTP bits can only go 0→1, NEVER back. This is **irreversible**
> and limited to a small number of write cycles. Most applications do NOT
> need OTP. Use only if you need persistent power-on defaults.

| Function | Description |
|----------|-------------|
| `tmc2209_otp_read(drv, &otp)` | Read all 3 OTP bytes (safe, no side effects) |
| `tmc2209_otp_program_bit(drv, byte, bit)` | Program single bit; verifies before/after |

### FACTORY_CONF

| Function | Description |
|----------|-------------|
| `tmc2209_get_factory_conf(drv, &fc)` | Read FCLKTRIM + OTTRIM |
| `tmc2209_set_fclktrim(drv, trim)` | Set clock trim 0..31 (affects UART timing) |

### Multi-device

| Function | Description |
|----------|-------------|
| `tmc2209_scan_bus(drv, results[4])` | Scan addresses 0..3, returns count found |

### Presets

| Function | Description |
|----------|-------------|
| `tmc2209_apply_stealthchop_defaults(drv)` | GCONF + PWMCONF for quiet StealthChop |
| `tmc2209_apply_spreadcycle_defaults(drv)` | GCONF + CHOPCONF for SpreadCycle |

### Utility

| Function | Description |
|----------|-------------|
| `tmc2209_last_error(drv)` | Last transport error code |
| `tmc2209_result_str(res)` | Error code to string |

## 6. Error Codes

All functions return `tmc2209_result_t`:

| Code | String | Meaning |
|------|--------|---------|
| `TMC2209_OK` | OK | Success |
| `TMC2209_ERR_TIMEOUT` | TIMEOUT | No UART response |
| `TMC2209_ERR_UART` | UART_ERROR | UART callback error |
| `TMC2209_ERR_BAD_FRAME` | BAD_FRAME | Response received but invalid |
| `TMC2209_ERR_CRC` | CRC_ERROR | CRC mismatch |
| `TMC2209_ERR_INVALID_ARG` | INVALID_ARG | NULL pointer or out-of-range parameter |
| `TMC2209_ERR_NOT_INIT` | NOT_INITIALIZED | Driver not initialized |
| `TMC2209_ERR_UNSUPPORTED` | UNSUPPORTED | Feature not available |
| `TMC2209_ERR_HW` | HW_ERROR | Hardware error (wrong VERSION, OTP fail) |

## 7. Shadow Registers

Write-only TMC2209 registers cannot be read back. The library maintains
shadow copies in `tmc2209_t.shadow` for these registers:

| Register | Shadow field | Access |
|----------|-------------|--------|
| GCONF | `shadow.gconf` | R/W (also readable from chip) |
| SLAVECONF | `shadow.slaveconf` | W only |
| IHOLD_IRUN | `shadow.ihold_irun` | W only |
| TPOWERDOWN | `shadow.tpowerdown` | W only |
| TPWMTHRS | `shadow.tpwmthrs` | W only |
| TCOOLTHRS | `shadow.tcoolthrs` | W only |
| VACTUAL | `shadow.vactual` | W only |
| SGTHRS | `shadow.sgthrs` | W only |
| COOLCONF | `shadow.coolconf` | W only |
| CHOPCONF | `shadow.chopconf` | R/W (synced on read) |
| PWMCONF | `shadow.pwmconf` | W only |
| FACTORY_CONF | `shadow.factory_conf` | R/W (synced on read) |

Shadows are populated during `tmc2209_init()`. Typed `set_*` functions
update the shadow atomically: modify → write → update shadow on success.

After a chip power-cycle or reset, call `tmc2209_init()` to re-sync.

## 8. Typed Config Structs

### tmc2209_chopconf_t

| Field | Range | Description |
|-------|-------|-------------|
| `toff` | 0..15 | Off-time (0=driver disabled, 1..15 = enabled) |
| `hstrt` | 0..7 | Hysteresis start value |
| `hend` | 0..15 | Hysteresis end value (offset -3 → actual -3..12) |
| `tbl` | 0..3 | Comparator blank time (16/24/36/54 clocks) |
| `vsense` | 0/1 | 0=low sensitivity (higher current range), 1=high |
| `mres` | 0..8 | Microstep resolution (0=256, 1=128, ..., 8=fullstep) |
| `intpol` | 0/1 | Interpolate to 256 microsteps |
| `dedge` | 0/1 | Count both STEP edges |
| `diss2g` | 0/1 | Disable short-to-GND protection |
| `diss2vs` | 0/1 | Disable short-to-VS protection |

### tmc2209_pwmconf_t

| Field | Range | Description |
|-------|-------|-------------|
| `pwm_ofs` | 0..255 | User-defined PWM amplitude offset |
| `pwm_grad` | 0..255 | Velocity-dependent PWM gradient |
| `pwm_freq` | 0..3 | PWM frequency (0=2/1024, 1=2/683, 2=2/512, 3=2/410 fclk) |
| `pwm_autoscale` | 0/1 | Enable automatic current scaling |
| `pwm_autograd` | 0/1 | Enable automatic gradient adaptation |
| `freewheel` | 0..3 | Standstill mode: 0=normal, 1=freewheel, 2=LS short, 3=HS short |
| `pwm_reg` | 0..15 | Regulation loop gradient |
| `pwm_lim` | 0..15 | PWM limit for switching back to fullstep |

### tmc2209_coolstep_config_t

| Field | Range | Description |
|-------|-------|-------------|
| `semin` | 0..15 | Minimum SG value to enable CoolStep (0=off) |
| `seup` | 0..3 | Current increment step (0=1, 1=2, 2=4, 3=8) |
| `semax` | 0..15 | Hysteresis for CoolStep upper threshold |
| `sedn` | 0..3 | Current decrement speed (0..3) |
| `seimin` | 0/1 | Minimum current: 0=half CS_ACTUAL, 1=quarter |

### tmc2209_drv_status_t

| Field | Description |
|-------|-------------|
| `otpw` | Over-temperature pre-warning (>120°C) |
| `ot` | Over-temperature shutdown (>150°C) |
| `s2ga/s2gb` | Short to GND detected (phase A/B) |
| `s2vsa/s2vsb` | Short to supply detected (phase A/B) |
| `ola/olb` | Open load detected (phase A/B) |
| `t120..t157` | Temperature threshold flags |
| `cs_actual` | Actual motor current scale 0..31 |
| `stealth` | StealthChop mode active |
| `stst` | Motor standstill detected |

## 9. Limitations and Caveats

- **OTP is irreversible.** Bits go 0→1 only, limited write cycles.
- **Standby/reset** requires `tmc2209_init()` to re-synchronize all registers.
- **DIAG/INDEX pin** behavior depends on board wiring. The library can configure
  `index_otpw` and `index_step` via GCONF, but the pin must be physically connected.
- **Internal Rsense** changes current scaling dramatically. Only enable when motor
  is disabled. Not all boards support this.
- **STEP/DIR mode** is not part of this library — it depends on platform timers.
- **Not thread-safe.** The debug buffer is static/shared. Do not call library
  functions from multiple threads without external synchronization.
- **FACTORY_CONF FCLKTRIM** affects the internal oscillator frequency. Changing it
  may affect UART timing if using internal clock. Modify only if you understand
  the implications.
- **Sensorless homing** via StallGuard requires DIAG pin connected to a controller
  input and appropriate TCOOLTHRS/SGTHRS tuning at the target velocity.

## 10. File Structure

```
lib/tmc2209/
├── include/tmc2209/
│   ├── tmc2209.h          ← public API + driver context
│   ├── tmc2209_types.h    ← result codes, configs, decoded register structs
│   ├── tmc2209_regs.h     ← register addresses and bitfield macros
│   └── tmc2209_port.h     ← platform I/O callback interface
├── src/
│   └── tmc2209.c          ← core implementation
└── library.json

docs/
├── TMC2209_API.md          ← this file
└── examples/
    ├── 01_basic_init.c
    ├── 02_motion_vactual.c
    ├── 03_diagnostics.c
    ├── 04_coolstep_stallguard.c
    ├── 05_multi_device.c
    └── 06_otp_advanced.c
```

## 11. Porting to a New Platform

1. Implement 5 callbacks in `tmc2209_io_t` for your hardware
2. Fill `tmc2209_config_t` with your motor parameters
3. Call `tmc2209_init()` — done

No `#define` board macros, no global state, no compile-time platform selection.
The entire platform dependency is isolated in the callback struct.

See `tmc2209_port_stm32_hal.c` for a complete STM32 HAL reference implementation.
