/* Example 3: Reading and decoding diagnostic registers. */

#include "tmc2209/tmc2209.h"
#include <stdio.h>

void example_diagnostics(tmc2209_t *drv)
{
    /* Clear any previous error flags */
    tmc2209_clear_gstat(drv);

    /* Read GSTAT */
    tmc2209_gstat_t gs;
    if (tmc2209_get_gstat(drv, &gs) == TMC2209_OK) {
        printf("GSTAT: reset=%u drv_err=%u uv_cp=%u\n",
               gs.reset, gs.drv_err, gs.uv_cp);
    }

    /* Read IOIN (pin states + version) */
    tmc2209_ioin_t ioin;
    if (tmc2209_get_ioin(drv, &ioin) == TMC2209_OK) {
        printf("IOIN: version=0x%02X enn=%u diag=%u\n",
               ioin.version, ioin.enn, ioin.diag);
    }

    /* Full driver status */
    tmc2209_drv_status_t ds;
    if (tmc2209_get_drv_status(drv, &ds) == TMC2209_OK) {
        printf("DRV_STATUS: cs_actual=%u stealth=%u stst=%u\n",
               ds.cs_actual, ds.stealth, ds.stst);
        if (ds.ot)   printf("  WARNING: over-temperature shutdown!\n");
        if (ds.otpw) printf("  WARNING: over-temperature pre-warning\n");
        if (ds.s2ga || ds.s2gb) printf("  WARNING: short to GND\n");
        if (ds.ola  || ds.olb)  printf("  WARNING: open load\n");
    }

    /* StallGuard result */
    uint16_t sg;
    if (tmc2209_get_sg_result(drv, &sg) == TMC2209_OK)
        printf("SG_RESULT=%u\n", sg);

    /* PWM autotune status */
    tmc2209_pwm_scale_t ps;
    if (tmc2209_get_pwm_scale(drv, &ps) == TMC2209_OK)
        printf("PWM_SCALE: sum=%u auto=%d\n", ps.pwm_scale_sum, ps.pwm_scale_auto);

    /* Phase currents */
    tmc2209_mscuract_t mc;
    if (tmc2209_get_mscuract(drv, &mc) == TMC2209_OK)
        printf("MSCURACT: cur_a=%d cur_b=%d\n", mc.cur_a, mc.cur_b);
}
