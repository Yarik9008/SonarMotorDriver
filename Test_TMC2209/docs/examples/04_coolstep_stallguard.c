/* Example 4: CoolStep adaptive current + StallGuard stall detection. */

#include "tmc2209/tmc2209.h"
#include <stdio.h>

void example_coolstep_stallguard(tmc2209_t *drv)
{
    /* StallGuard: detect stall when SG_RESULT falls below threshold×2.
     * Higher sgthrs = more sensitive (stall detected earlier). */
    tmc2209_stallguard_config_t sg = {
        .sgthrs    = 100,    /* Stall threshold */
        .tcoolthrs = 0xFFFFF /* CoolStep/SG active at all velocities */
    };
    tmc2209_configure_stallguard(drv, &sg);

    /* CoolStep: automatically reduce current when load is low.
     * semin > 0 enables CoolStep. */
    tmc2209_coolstep_config_t cs = {
        .semin  = 5,  /* Enable CoolStep when SG_RESULT < semin×32 */
        .seup   = 2,  /* Current increment step size (1=1, 2=2, 3=4, 4=8) */
        .semax  = 2,  /* Hysteresis: disable CoolStep when SG > (semin+semax+1)×32 */
        .sedn   = 1,  /* Current decrement speed */
        .seimin = 0,  /* 0 = half CS for minimum, 1 = quarter CS */
    };
    tmc2209_set_coolstep_config(drv, &cs);

    tmc2209_enable(drv);
    tmc2209_set_vactual(drv, 5000);

    /* Monitor StallGuard during motion */
    uint16_t sg_result;
    uint8_t  cs_actual;
    tmc2209_get_sg_result(drv, &sg_result);
    tmc2209_get_cs_actual(drv, &cs_actual);
    printf("SG=%u CS=%u\n", sg_result, cs_actual);

    /* To disable CoolStep: set semin=0 */
    cs.semin = 0;
    tmc2209_set_coolstep_config(drv, &cs);
}
