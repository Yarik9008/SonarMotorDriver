/* Пример 4: адаптивный ток CoolStep и обнаружение заклинивания StallGuard. */

#include "tmc2209/tmc2209.h"
#include <stdio.h>

void example_coolstep_stallguard(tmc2209_t *drv)
{
    /* StallGuard: заклинивание, когда SG_RESULT опускается ниже порога×2.
     * Большее sgthrs — выше чувствительность (заклинивание раньше). */
    tmc2209_stallguard_config_t sg = {
        .sgthrs    = 100,    /* Порог заклинивания (0..255) */
        .tcoolthrs = 0xFFFFF /* CoolStep/SG активны на всех скоростях (макс. 20 бит) */
    };
    if (tmc2209_configure_stallguard(drv, &sg) != TMC2209_OK) return;

    /* CoolStep: автоматическое снижение тока при малой нагрузке.
     * semin > 0 включает CoolStep. */
    tmc2209_coolstep_config_t cs = {
        .semin  = 5,  /* Включить CoolStep при SG_RESULT < semin×32 */
        .seup   = 2,  /* Шаг увеличения тока (1=1, 2=2, 3=4, 4=8) */
        .semax  = 2,  /* Гистерезис: выкл. CoolStep при SG > (semin+semax+1)×32 */
        .sedn   = 1,  /* Скорость уменьшения тока */
        .seimin = 0,  /* 0 = полов. CS минимум, 1 = четверть CS */
    };
    if (tmc2209_set_coolstep_config(drv, &cs) != TMC2209_OK) return;

    tmc2209_enable(drv);
    tmc2209_set_vactual(drv, 5000);

    /* Наблюдение за StallGuard во время движения */
    uint16_t sg_result;
    uint8_t  cs_actual;
    tmc2209_get_sg_result(drv, &sg_result);
    tmc2209_get_cs_actual(drv, &cs_actual);
    printf("SG=%u CS=%u\n", sg_result, cs_actual);

    /* Чтобы выключить CoolStep: установить semin=0 */
    cs.semin = 0;
    tmc2209_set_coolstep_config(drv, &cs);
}
