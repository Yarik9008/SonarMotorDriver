/* tmc2209.h — App-level wrapper for TMC2209 library (SonarMotorDriver).
 *
 * Provides a simple facade over the reusable TMC2209 library for use in
 * the SonarMotorDriver firmware.  STEP/DIR motion is handled by stepper.c;
 * this module owns only driver configuration, enable, and diagnostics.
 */

#ifndef TMC2209_APP_H
#define TMC2209_APP_H

#include <stdint.h>
#include "tmc2209/tmc2209.h"

typedef enum {
    TMC_BUSY = 0,
    TMC_DONE,
    TMC_ERROR
} TMC2209_Status;

/* ---- Legacy-compatible init API (used by Init_Poll state machine) ---- */

void           TMC2209_InitStart(void);
TMC2209_Status TMC2209_Poll(void);

/* ---- State ---- */

uint8_t TMC2209_IsReady(void);

/* ---- Enable / disable (writes ENN pin via library port layer) ---- */

void TMC2209_SetEnabled(uint8_t enabled);

/* ---- Runtime reconfiguration ---- */

TMC2209_Status TMC2209_SetCurrent(uint16_t run_ma, uint16_t hold_ma);
TMC2209_Status TMC2209_SetMicrosteps(uint16_t ms);

/* ---- Diagnostics ---- */

TMC2209_Status TMC2209_GetDrvStatus(tmc2209_drv_status_t *st);
TMC2209_Status TMC2209_GetVersion(uint8_t *version);

/* ---- Direct library access (advanced use) ---- */

tmc2209_t *TMC2209_GetDriver(void);

#endif /* TMC2209_APP_H */
