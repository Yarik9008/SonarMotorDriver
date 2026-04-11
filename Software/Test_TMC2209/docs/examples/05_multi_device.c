/* Пример 5: несколько TMC2209 на одной шине UART (адреса 0..3).
 *
 * Все устройства используют одни линии UART TX/RX.
 * Адрес задаётся выводами MS1/MS2:
 *   MS2=0 MS1=0 → адрес 0
 *   MS2=0 MS1=1 → адрес 1
 *   MS2=1 MS1=0 → адрес 2
 *   MS2=1 MS1=1 → адрес 3
 *
 * Используйте отдельные экземпляры tmc2209_t с разными cfg.addr.
 * Общий один tmc2209_io_t (один UART, одни колбэки).
 */

#include "tmc2209/tmc2209.h"
#include <stdio.h>
#include <string.h>

void example_multi_device(tmc2209_io_t *shared_io)
{
    /* Сканирование шины: для scan_bus нужны только cfg и io (tmc2209_init не вызываем). */
    tmc2209_t scanner;
    memset(&scanner, 0, sizeof(scanner));
    scanner.cfg = TMC2209_DEFAULT_CONFIG;
    scanner.io  = *shared_io;

    tmc2209_scan_entry_t entries[4];
    uint8_t found = tmc2209_scan_bus(&scanner, entries);
    printf("На шине найдено %u TMC2209\n", (unsigned)found);
    for (unsigned i = 0; i < 4; i++) {
        if (entries[i].found)
            printf("  адрес %u: version=0x%02X\n", entries[i].addr, entries[i].version);
    }

    /* Инициализация каждого обнаруженного устройства */
    tmc2209_t motors[4];
    memset(motors, 0, sizeof(motors));
    for (unsigned i = 0; i < 4; i++) {
        if (!entries[i].found) continue;

        tmc2209_config_t cfg = TMC2209_DEFAULT_CONFIG;
        cfg.addr    = entries[i].addr;
        cfg.irun_ma = 600;

        tmc2209_result_t res = tmc2209_init(&motors[i], &cfg, shared_io);
        if (res == TMC2209_OK) {
            tmc2209_enable(&motors[i]);
            printf("Двигатель %u инициализирован и включён\n", i);
        }
    }

    /* Управление двигателями (только если они были найдены и инициализированы) */
    if (entries[0].found) tmc2209_set_vactual(&motors[0], 1000);
    if (entries[1].found) tmc2209_set_vactual(&motors[1], -500);
}
