/* Example 5: Multiple TMC2209 on one UART bus (addresses 0..3).
 *
 * All devices share the same UART TX/RX lines.
 * Each device has a unique address set by MS1/MS2 pins:
 *   MS2=0 MS1=0 → addr 0
 *   MS2=0 MS1=1 → addr 1
 *   MS2=1 MS1=0 → addr 2
 *   MS2=1 MS1=1 → addr 3
 *
 * Use separate tmc2209_t instances with different cfg.addr values.
 * All share the same tmc2209_io_t (same UART, same callbacks).
 */

#include "tmc2209/tmc2209.h"
#include <stdio.h>

void example_multi_device(tmc2209_io_t *shared_io)
{
    /* Scan bus first to discover connected devices */
    tmc2209_t scanner;
    tmc2209_config_t scan_cfg = TMC2209_DEFAULT_CONFIG;
    /* Minimal init just for scanning — we need io callbacks set up */
    memset(&scanner, 0, sizeof(scanner));
    scanner.cfg = scan_cfg;
    scanner.io  = *shared_io;

    tmc2209_scan_entry_t entries[4];
    uint8_t found = tmc2209_scan_bus(&scanner, entries);
    printf("Found %u TMC2209 on bus\n", found);
    for (int i = 0; i < 4; i++) {
        if (entries[i].found)
            printf("  addr %u: version=0x%02X\n", entries[i].addr, entries[i].version);
    }

    /* Initialize each discovered device */
    tmc2209_t motors[4];
    for (int i = 0; i < 4; i++) {
        if (!entries[i].found) continue;

        tmc2209_config_t cfg = TMC2209_DEFAULT_CONFIG;
        cfg.addr    = entries[i].addr;
        cfg.irun_ma = 600;

        tmc2209_result_t res = tmc2209_init(&motors[i], &cfg, shared_io);
        if (res == TMC2209_OK) {
            tmc2209_enable(&motors[i]);
            printf("Motor %u initialized and enabled\n", i);
        }
    }

    /* Control each motor independently */
    tmc2209_set_vactual(&motors[0], 1000);
    tmc2209_set_vactual(&motors[1], -500);
}
