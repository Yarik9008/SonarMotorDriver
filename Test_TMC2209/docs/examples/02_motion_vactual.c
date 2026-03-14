/* Example 2: Motor motion via VACTUAL (internal step generator). */

#include "tmc2209/tmc2209.h"

void example_motion(tmc2209_t *drv)
{
    /* Configure motor parameters */
    tmc2209_set_current(drv, 1000, 500);   /* 1000mA run, 500mA hold */
    tmc2209_set_microsteps(drv, 32);       /* 32 microsteps */

    tmc2209_enable(drv);

    /* Move forward at velocity 5000 */
    tmc2209_set_vactual(drv, 5000);

    /* ... wait or do other work ... */

    /* Reverse direction */
    tmc2209_set_vactual(drv, -3000);

    /* ... */

    /* Gradual deceleration */
    tmc2209_set_vactual(drv, -1000);
    tmc2209_set_vactual(drv, -200);
    tmc2209_stop(drv);

    /* Change current on the fly */
    tmc2209_set_run_current(drv, 600);

    tmc2209_disable(drv);
}
