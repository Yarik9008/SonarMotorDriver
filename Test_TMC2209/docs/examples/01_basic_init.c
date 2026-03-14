/* Example 1: Basic initialization, version read, enable/disable.
 * Reference code — adapt platform callbacks for your hardware. */

#include "tmc2209/tmc2209.h"

/* Platform callbacks (implement for your MCU) */
extern int  my_uart_tx(const uint8_t *d, uint16_t len, uint32_t tmo, void *ctx);
extern int  my_uart_rx(uint8_t *d, uint16_t max, uint32_t tmo, uint16_t *n, void *ctx);
extern void my_uart_flush(void *ctx);
extern void my_delay_us(uint32_t us, void *ctx);
extern void my_set_enable(uint8_t level, void *ctx);

void example_basic_init(void)
{
    tmc2209_config_t cfg = TMC2209_DEFAULT_CONFIG;
    cfg.addr     = 0;
    cfg.rsense   = 0.11f;
    cfg.irun_ma  = 800;
    cfg.ihold_ma = 400;

    tmc2209_io_t io = {
        .uart_tx       = my_uart_tx,
        .uart_rx       = my_uart_rx,
        .uart_rx_flush = my_uart_flush,
        .delay_us      = my_delay_us,
        .set_enable    = my_set_enable,
        .debug_print   = 0,
        .ctx           = 0,
    };

    tmc2209_t drv;
    tmc2209_result_t res = tmc2209_init(&drv, &cfg, &io);
    if (res != TMC2209_OK) {
        /* Handle error: tmc2209_result_str(res) */
        return;
    }

    uint8_t version = 0;
    tmc2209_get_version(&drv, &version);
    /* version should be 0x21 */

    tmc2209_enable(&drv);
    /* Motor is now enabled with configured current */

    /* ... do work ... */

    tmc2209_stop(&drv);
    tmc2209_disable(&drv);
    tmc2209_deinit(&drv);
}
