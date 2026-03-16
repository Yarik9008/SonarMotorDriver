/* Пример 3: чтение и декодирование диагностических регистров. */

#include "tmc2209/tmc2209.h"
#include <stdio.h>

void example_diagnostics(tmc2209_t *drv)
{
    /* Сброс предыдущих флагов ошибок */
    tmc2209_clear_gstat(drv);

    /* Чтение GSTAT */
    tmc2209_gstat_t gs;
    if (tmc2209_get_gstat(drv, &gs) == TMC2209_OK) {
        printf("GSTAT: reset=%u drv_err=%u uv_cp=%u\n",
               gs.reset, gs.drv_err, gs.uv_cp);
    }

    /* Чтение IOIN (состояния выводов и версия) */
    tmc2209_ioin_t ioin;
    if (tmc2209_get_ioin(drv, &ioin) == TMC2209_OK) {
        printf("IOIN: version=0x%02X enn=%u diag=%u\n",
               ioin.version, ioin.enn, ioin.diag);
    }

    /* Полный статус драйвера */
    tmc2209_drv_status_t ds;
    if (tmc2209_get_drv_status(drv, &ds) == TMC2209_OK) {
        printf("DRV_STATUS: cs_actual=%u stealth=%u stst=%u\n",
               ds.cs_actual, ds.stealth, ds.stst);
        if (ds.ot)   printf("  ВНИМАНИЕ: отключение по перегреву!\n");
        if (ds.otpw) printf("  ВНИМАНИЕ: предупреждение о перегреве\n");
        if (ds.s2ga || ds.s2gb) printf("  ВНИМАНИЕ: КЗ на GND\n");
        if (ds.ola  || ds.olb)  printf("  ВНИМАНИЕ: обрыв нагрузки\n");
    }

    /* Результат StallGuard */
    uint16_t sg;
    if (tmc2209_get_sg_result(drv, &sg) == TMC2209_OK)
        printf("SG_RESULT=%u\n", sg);

    /* Статус автонастройки ШИМ */
    tmc2209_pwm_scale_t ps;
    if (tmc2209_get_pwm_scale(drv, &ps) == TMC2209_OK)
        printf("PWM_SCALE: sum=%u auto=%d\n", ps.pwm_scale_sum, ps.pwm_scale_auto);

    /* Токи фаз */
    tmc2209_mscuract_t mc;
    if (tmc2209_get_mscuract(drv, &mc) == TMC2209_OK)
        printf("MSCURACT: cur_a=%d cur_b=%d\n", mc.cur_a, mc.cur_b);
}
