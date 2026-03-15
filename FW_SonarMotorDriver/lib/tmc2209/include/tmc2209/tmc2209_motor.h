/* tmc2209_motor.h — High-level Motor Facade API for TMC2209.
 *
 * This layer manages the motor state, control modes, and backends 
 * (STEP/DIR or UART motion).
 */

#ifndef TMC2209_MOTOR_H
#define TMC2209_MOTOR_H

#include <stdint.h>
#include "tmc2209_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Control modes */
typedef enum {
    TMC2209_MOTOR_CONTROL_STEP_DIR = 0,
    TMC2209_MOTOR_CONTROL_UART     = 1
} tmc2209_motor_mode_t;

/* Motor configuration snapshot */
typedef struct {
    uint16_t             run_ma;
    uint16_t             hold_ma;
    uint16_t             microsteps;
    tmc2209_motor_mode_t mode;
    uint8_t              ready;
} tmc2209_motor_config_t;

/* ---- Initialization & Task ---- */

/** 
 * Initialize motor subsystem. 
 * Returns 0 on success, <0 on error. 
 */
int  tmc2209_motor_init(void);

/** 
 * Periodic task to be called from main loop. 
 * Handles non-blocking diagnostics and background maintenance.
 */
void tmc2209_motor_task(void);

/** Returns 1 if motor subsystem is fully initialized and communication is verified. */
int  tmc2209_motor_is_ready(void);

/* ---- Basic Control ---- */

/** 
 * Enable or disable the driver (controls ENN pin). 
 * Returns: 0=OK, -1=Not ready, -2=Bad arg, -3=Apply failed.
 */
int tmc2209_motor_set_enabled(int enabled);

/** Stop all movement immediately. */
void tmc2209_motor_stop(void);

/** 
 * Returns 1 if motor is in "commanded-moving" state.
 * (PWM pulses are active or VACTUAL is non-zero).
 * Note: This does not guarantee physical movement if driver is disabled or stalled.
 */
int  tmc2209_motor_is_moving(void);

/* ---- Motion Commands ---- */

/** 
 * Set target movement rate ("steps" per control loop tick).
 * Used by the control loop for continuous motion.
 * In STEP/DIR: sets pulse frequency.
 * In UART: sets VACTUAL.
 */
void tmc2209_motor_move_steps(int32_t steps);

/** 
 * Move with target velocity (UART mode only). 
 */
void tmc2209_motor_move_velocity(int32_t velocity);

/* ---- Configuration ---- */

/** 
 * Set run and hold current in mA. 
 * Returns: 0=OK, -1=Not ready, -2=Bad arg, -3=Apply failed.
 */
int  tmc2209_motor_set_current(uint16_t run_ma, uint16_t hold_ma);

/** 
 * Set microstep resolution (1, 2, 4, ..., 256). 
 * Returns: 0=OK, -1=Not ready, -2=Bad arg, -3=Apply failed.
 */
int  tmc2209_motor_set_microsteps(uint16_t microsteps);

/** Get current motor configuration. */
void tmc2209_motor_get_config(tmc2209_motor_config_t *cfg);

/** Get control mode (STEP/DIR or UART). */
tmc2209_motor_mode_t tmc2209_motor_get_control_mode(void);

/* ---- Diagnostics ---- */

/** Get chip version. */
int  tmc2209_motor_get_version(uint8_t *version);

/** Get full driver status (blocking UART read). */
int  tmc2209_motor_get_drv_status(tmc2209_drv_status_t *st);

/** Get cached driver status (non-blocking, updated by task). */
int  tmc2209_motor_get_cached_drv_status(tmc2209_drv_status_t *st);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_MOTOR_H */
