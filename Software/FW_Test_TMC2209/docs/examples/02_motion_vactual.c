/* Пример 2: движение двигателя через VACTUAL (внутренний генератор шагов). */

#include "tmc2209/tmc2209.h"

void example_motion(tmc2209_t *drv)
{
    /* Настройка параметров двигателя (в реальном коде проверяйте возврат tmc2209_result_t) */
    tmc2209_set_current(drv, 1000, 500);   /* 1000 мА ток движения, 500 мА удержания */
    tmc2209_set_microsteps(drv, 32);        /* 32 микрошага */

    tmc2209_enable(drv);

    /* Движение вперёд со скоростью 5000 */
    tmc2209_set_vactual(drv, 5000);

    /* ... ожидание или другая работа ... */

    /* Направление назад */
    tmc2209_set_vactual(drv, -3000);

    /* ... */

    /* Плавное замедление */
    tmc2209_set_vactual(drv, -1000);
    tmc2209_set_vactual(drv, -200);
    tmc2209_stop(drv);

    /* Изменение тока на ходу */
    tmc2209_set_run_current(drv, 600);

    tmc2209_disable(drv);
}
