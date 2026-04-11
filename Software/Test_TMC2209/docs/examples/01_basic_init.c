/* Пример 1: базовая инициализация, чтение версии, включение/выключение.
 * Эталонный код — адаптируйте платформенные колбэки под своё железо. */

#include "tmc2209/tmc2209.h"
#include <stddef.h>

/* Платформенные колбэки (реализуйте под свой МК, сигнатуры — как в tmc2209_io_t) */
extern int  my_uart_tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms, void *ctx);
extern int  my_uart_rx(uint8_t *data, uint16_t max_len, uint32_t timeout_ms, uint16_t *received, void *ctx);
extern void my_uart_rx_flush(void *ctx);
extern void my_delay_us(uint32_t us, void *ctx);
extern void my_set_enable(uint8_t level, void *ctx);

void example_basic_init(void)
{
    tmc2209_config_t cfg = TMC2209_DEFAULT_CONFIG;
    cfg.addr           = 0;
    cfg.rsense         = 0.11f;
    cfg.irun_ma        = 800;
    cfg.ihold_ma       = 400;
    cfg.reply_delay_us = 500;

    tmc2209_io_t io = {
        .uart_tx       = my_uart_tx,
        .uart_rx       = my_uart_rx,
        .uart_rx_flush = my_uart_rx_flush,
        .delay_us      = my_delay_us,
        .set_enable    = my_set_enable,
        .debug_print   = NULL,
        .ctx           = NULL,
    };

    tmc2209_t drv;
    tmc2209_result_t res = tmc2209_init(&drv, &cfg, &io);
    if (res != TMC2209_OK) {
        /* Ошибка инициализации: tmc2209_result_str(res) */
        return;
    }

    uint8_t version = 0;
    if (tmc2209_get_version(&drv, &version) == TMC2209_OK) {
        /* version обычно 0x21 для TMC2209 */
    }

    tmc2209_enable(&drv);
    /* Двигатель включён с настроенным током */

    /* ... выполнение работы ... */

    tmc2209_stop(&drv);
    tmc2209_disable(&drv);
    tmc2209_deinit(&drv);
}
