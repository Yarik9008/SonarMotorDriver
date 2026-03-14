/* tmc2209.h — TMC2209 stepper motor driver library: public API.
 *
 * Platform-agnostic UART driver for Trinamic TMC2209.
 * Provides register-level access and high-level motor configuration.
 *
 * Usage:
 *   1. Implement tmc2209_io_t callbacks for your platform
 *   2. Fill tmc2209_config_t (or use TMC2209_DEFAULT_CONFIG)
 *   3. Call tmc2209_init()
 *   4. Call tmc2209_enable() and control the motor
 */

#ifndef TMC2209_H
#define TMC2209_H

#include "tmc2209/tmc2209_types.h"
#include "tmc2209/tmc2209_regs.h"
#include "tmc2209/tmc2209_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Driver context ---- */

typedef struct {
    tmc2209_config_t cfg;
    tmc2209_io_t     io;
    tmc2209_result_t last_error;
    uint8_t          initialized;
} tmc2209_t;

/* ---- Init / deinit ----
 *
 * tmc2209_init performs a blocking initialization sequence:
 *   - writes GCONF, IHOLD_IRUN, TPWMTHRS, SLAVECONF
 *   - verifies communication (reads IFCNT and IOIN/VERSION)
 *   - configures CHOPCONF (microsteps) and PWMCONF
 *   - sets VACTUAL=0
 *   - does NOT enable the driver (call tmc2209_enable afterwards)
 */
tmc2209_result_t tmc2209_init(tmc2209_t *drv, const tmc2209_config_t *cfg,
                              const tmc2209_io_t *io);
void             tmc2209_deinit(tmc2209_t *drv);

/* ---- Low-level register access ---- */

tmc2209_result_t tmc2209_read_reg(tmc2209_t *drv, uint8_t reg, uint32_t *value);
tmc2209_result_t tmc2209_write_reg(tmc2209_t *drv, uint8_t reg, uint32_t value);

/* Read register at an arbitrary slave address (for address scanning). */
tmc2209_result_t tmc2209_read_reg_addr(tmc2209_t *drv, uint8_t addr,
                                       uint8_t reg, uint32_t *value);

/* ---- Configuration ---- */

tmc2209_result_t tmc2209_set_current(tmc2209_t *drv, uint16_t run_ma, uint16_t hold_ma);
tmc2209_result_t tmc2209_set_microsteps(tmc2209_t *drv, uint16_t ms);
tmc2209_result_t tmc2209_set_chopconf(tmc2209_t *drv, uint32_t value);
tmc2209_result_t tmc2209_set_pwmconf(tmc2209_t *drv, uint32_t value);

/* ---- Motor control ---- */

tmc2209_result_t tmc2209_enable(tmc2209_t *drv);
tmc2209_result_t tmc2209_disable(tmc2209_t *drv);
tmc2209_result_t tmc2209_set_vactual(tmc2209_t *drv, int32_t velocity);
tmc2209_result_t tmc2209_stop(tmc2209_t *drv);

/* ---- Diagnostics (decoded) ---- */

tmc2209_result_t tmc2209_get_version(tmc2209_t *drv, uint8_t *version);
tmc2209_result_t tmc2209_get_ifcnt(tmc2209_t *drv, uint8_t *count);
tmc2209_result_t tmc2209_get_ioin(tmc2209_t *drv, tmc2209_ioin_t *ioin);
tmc2209_result_t tmc2209_get_drv_status(tmc2209_t *drv, tmc2209_drv_status_t *status);
tmc2209_result_t tmc2209_get_gstat(tmc2209_t *drv, tmc2209_gstat_t *gstat);
tmc2209_result_t tmc2209_get_sg_result(tmc2209_t *drv, uint16_t *result);
tmc2209_result_t tmc2209_get_tstep(tmc2209_t *drv, uint32_t *tstep);

/* ---- Utility ---- */

tmc2209_result_t tmc2209_last_error(const tmc2209_t *drv);
const char      *tmc2209_result_str(tmc2209_result_t res);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_H */
