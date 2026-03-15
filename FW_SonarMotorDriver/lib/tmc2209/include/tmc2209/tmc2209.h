/**
 * @file tmc2209.h
 * @brief Основной интерфейс библиотеки для работы с чипом TMC2209.
 *
 * Предоставляет функции для инициализации драйвера, чтения и записи регистров,
 * управления током, микрошагом, режимами StealthChop/SpreadCycle, а также
 * функции диагностики (StallGuard, DRV_STATUS).
 * Библиотека не зависит от платформы; взаимодействие с железом происходит через
 * колбэки структуры tmc2209_io_t.
 */

#ifndef TMC2209_H
#define TMC2209_H

#include "tmc2209/tmc2209_types.h"
#include "tmc2209/tmc2209_regs.h"
#include "tmc2209/tmc2209_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==== Driver context ==== */

typedef struct {
    tmc2209_config_t cfg;
    tmc2209_io_t     io;
    tmc2209_result_t last_error;
    uint8_t          initialized;
    tmc2209_shadow_t shadow;
    struct {
        uint32_t ihold_irun_saved;
        uint32_t tpowerdown_saved;
        uint8_t  active;
    } standby;
} tmc2209_t;

/* ==== Init / deinit ==== */

/** Blocking init: configures all registers from cfg, verifies communication. */
tmc2209_result_t tmc2209_init(tmc2209_t *drv, const tmc2209_config_t *cfg,
                              const tmc2209_io_t *io);
void             tmc2209_deinit(tmc2209_t *drv);

/* ==== Low-level register access ==== */

tmc2209_result_t tmc2209_read_reg(tmc2209_t *drv, uint8_t reg, uint32_t *value);
tmc2209_result_t tmc2209_write_reg(tmc2209_t *drv, uint8_t reg, uint32_t value);
tmc2209_result_t tmc2209_read_reg_addr(tmc2209_t *drv, uint8_t addr,
                                       uint8_t reg, uint32_t *value);

/* ==== GCONF ==== */

/** Read GCONF from chip and decode. */
tmc2209_result_t tmc2209_get_gconf(tmc2209_t *drv, tmc2209_gconf_t *gc);
/** Invert motor shaft direction. */
tmc2209_result_t tmc2209_set_shaft(tmc2209_t *drv, uint8_t inverted);
/** Switch to SpreadCycle mode. */
tmc2209_result_t tmc2209_enable_spreadcycle(tmc2209_t *drv);
/** Switch to StealthChop mode. */
tmc2209_result_t tmc2209_enable_stealthchop(tmc2209_t *drv);
/**
 * Enable internal sense resistors.
 * WARNING: changes current scaling; apply before enabling motor.
 */
tmc2209_result_t tmc2209_enable_internal_rsense(tmc2209_t *drv, uint8_t enable);

/* ==== Current / IHOLD_IRUN ==== */

/** Set both run and hold current in mA (converts to CS using rsense). */
tmc2209_result_t tmc2209_set_current(tmc2209_t *drv, uint16_t run_ma, uint16_t hold_ma);
/** Set run current only (hold current preserved). */
tmc2209_result_t tmc2209_set_run_current(tmc2209_t *drv, uint16_t run_ma);
/** Set hold current only (run current preserved). */
tmc2209_result_t tmc2209_set_hold_current(tmc2209_t *drv, uint16_t hold_ma);
/** Get current CS values from shadow register. */
tmc2209_result_t tmc2209_get_current_config(tmc2209_t *drv, tmc2209_current_config_t *cc);
/** Set IHOLDDELAY (0..15). */
tmc2209_result_t tmc2209_set_iholddelay(tmc2209_t *drv, uint8_t delay);
/** Set TPOWERDOWN register (0..255). */
tmc2209_result_t tmc2209_set_tpowerdown(tmc2209_t *drv, uint8_t value);

/* ==== CHOPCONF ==== */

/** Set CHOPCONF from typed struct. Updates shadow. */
tmc2209_result_t tmc2209_set_chopconf_config(tmc2209_t *drv, const tmc2209_chopconf_t *cc);
/** Read CHOPCONF from chip and decode. */
tmc2209_result_t tmc2209_get_chopconf_config(tmc2209_t *drv, tmc2209_chopconf_t *cc);
/** Set CHOPCONF from raw 32-bit value. */
tmc2209_result_t tmc2209_set_chopconf(tmc2209_t *drv, uint32_t value);
/** Set microstep resolution (1,2,4,8,16,32,64,128,256). */
tmc2209_result_t tmc2209_set_microsteps(tmc2209_t *drv, uint16_t ms);
/** Read current microstep resolution from chip. */
tmc2209_result_t tmc2209_get_microsteps(tmc2209_t *drv, uint16_t *ms);
/** Enable/disable 256-microstep interpolation. */
tmc2209_result_t tmc2209_enable_interpolation(tmc2209_t *drv, uint8_t enable);
/** Enable/disable double-edge STEP (both rising and falling). */
tmc2209_result_t tmc2209_enable_double_edge_step(tmc2209_t *drv, uint8_t enable);

/* ==== PWMCONF ==== */

/** Set PWMCONF from typed struct (write-only register, uses shadow). */
tmc2209_result_t tmc2209_set_pwmconf_config(tmc2209_t *drv, const tmc2209_pwmconf_t *pc);
/** Get current PWMCONF from shadow (register is write-only). */
tmc2209_result_t tmc2209_get_pwmconf_config(tmc2209_t *drv, tmc2209_pwmconf_t *pc);
/** Set PWMCONF from raw 32-bit value. */
tmc2209_result_t tmc2209_set_pwmconf(tmc2209_t *drv, uint32_t value);
/** Set freewheel / standstill mode. */
tmc2209_result_t tmc2209_set_freewheel(tmc2209_t *drv, tmc2209_freewheel_t mode);

/* ==== CoolStep ==== */

/** Configure CoolStep from typed struct. Writes COOLCONF shadow. */
tmc2209_result_t tmc2209_set_coolstep_config(tmc2209_t *drv,
                                             const tmc2209_coolstep_config_t *cs);
/** Get CoolStep config from shadow (COOLCONF is write-only). */
tmc2209_result_t tmc2209_get_coolstep_config(tmc2209_t *drv, tmc2209_coolstep_config_t *cs);
/** Set TCOOLTHRS velocity threshold for CoolStep/StallGuard. */
tmc2209_result_t tmc2209_set_tcoolthrs(tmc2209_t *drv, uint32_t threshold);

/* ==== StallGuard ==== */

/** Set StallGuard threshold (0..255). */
tmc2209_result_t tmc2209_set_sgthrs(tmc2209_t *drv, uint8_t threshold);
/** Get StallGuard threshold from shadow (SGTHRS is write-only). */
tmc2209_result_t tmc2209_get_sgthrs(tmc2209_t *drv, uint8_t *threshold);
/** Convenience: configure StallGuard thresholds and TCOOLTHRS together. */
tmc2209_result_t tmc2209_configure_stallguard(tmc2209_t *drv,
                                              const tmc2209_stallguard_config_t *sg);

/* ==== Motor control ==== */

tmc2209_result_t tmc2209_enable(tmc2209_t *drv);
tmc2209_result_t tmc2209_disable(tmc2209_t *drv);
tmc2209_result_t tmc2209_set_vactual(tmc2209_t *drv, int32_t velocity);
tmc2209_result_t tmc2209_stop(tmc2209_t *drv);

/* ==== Standby ==== */

/**
 * Enter software standby: disables driver, zeroes hold current,
 * sets freewheel mode. Previous settings are saved for restore.
 */
tmc2209_result_t tmc2209_enter_standby(tmc2209_t *drv);
/**
 * Exit standby: restores previous current/power-down settings
 * and re-enables the driver.
 */
tmc2209_result_t tmc2209_exit_standby(tmc2209_t *drv);

/* ==== Diagnostics ==== */

tmc2209_result_t tmc2209_get_version(tmc2209_t *drv, uint8_t *version);
tmc2209_result_t tmc2209_get_ifcnt(tmc2209_t *drv, uint8_t *count);
tmc2209_result_t tmc2209_get_ioin(tmc2209_t *drv, tmc2209_ioin_t *ioin);
tmc2209_result_t tmc2209_get_drv_status(tmc2209_t *drv, tmc2209_drv_status_t *st);
tmc2209_result_t tmc2209_get_gstat(tmc2209_t *drv, tmc2209_gstat_t *gs);
/** Clear GSTAT flags by writing 1 to each bit. */
tmc2209_result_t tmc2209_clear_gstat(tmc2209_t *drv);
tmc2209_result_t tmc2209_get_sg_result(tmc2209_t *drv, uint16_t *result);
tmc2209_result_t tmc2209_get_tstep(tmc2209_t *drv, uint32_t *tstep);
/** Read CS_ACTUAL from DRV_STATUS (0..31). */
tmc2209_result_t tmc2209_get_cs_actual(tmc2209_t *drv, uint8_t *cs);
tmc2209_result_t tmc2209_get_pwm_scale(tmc2209_t *drv, tmc2209_pwm_scale_t *ps);
tmc2209_result_t tmc2209_get_pwm_auto(tmc2209_t *drv, tmc2209_pwm_auto_t *pa);
tmc2209_result_t tmc2209_get_mscnt(tmc2209_t *drv, uint16_t *count);
tmc2209_result_t tmc2209_get_mscuract(tmc2209_t *drv, tmc2209_mscuract_t *mc);

/* ==== OTP ====
 *
 * WARNING: OTP bits can only be programmed from 0 to 1.
 * This operation is IRREVERSIBLE and limited to a few write cycles.
 * Read OTP first, only program if you fully understand the implications.
 */

/** Read all 3 OTP bytes. */
tmc2209_result_t tmc2209_otp_read(tmc2209_t *drv, tmc2209_otp_t *otp);
/**
 * Program a single OTP bit. byte_num=0..2, bit_num=0..7.
 * Reads before/after to verify. Returns ERR_HW if verification fails.
 */
tmc2209_result_t tmc2209_otp_program_bit(tmc2209_t *drv,
                                         uint8_t byte_num, uint8_t bit_num);

/* ==== FACTORY_CONF ==== */

tmc2209_result_t tmc2209_get_factory_conf(tmc2209_t *drv, tmc2209_factory_conf_t *fc);
/** Set internal clock trim (0..31). Affects UART timing with internal oscillator. */
tmc2209_result_t tmc2209_set_fclktrim(tmc2209_t *drv, uint8_t trim);

/* ==== Multi-device ==== */

/**
 * Scan UART bus for TMC2209 devices on addresses 0..3.
 * results must point to an array of 4 tmc2209_scan_entry_t.
 * Returns number of devices found.
 */
uint8_t tmc2209_scan_bus(tmc2209_t *drv, tmc2209_scan_entry_t results[4]);

/* ==== Presets ==== */

/** Apply StealthChop defaults (GCONF + PWMCONF + CHOPCONF.toff). */
tmc2209_result_t tmc2209_apply_stealthchop_defaults(tmc2209_t *drv);
/** Apply SpreadCycle defaults (GCONF + CHOPCONF). */
tmc2209_result_t tmc2209_apply_spreadcycle_defaults(tmc2209_t *drv);

/* ==== Utility ==== */

tmc2209_result_t tmc2209_last_error(const tmc2209_t *drv);
const char      *tmc2209_result_str(tmc2209_result_t res);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_H */
