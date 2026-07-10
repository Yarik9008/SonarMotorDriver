/* tmc2209_port.h — слой абстракции платформы для библиотеки TMC2209.
 *
 * Реализуйте эти колбэки для своей платформы и передайте их
 * в tmc2209_init() через tmc2209_io_t. */

#ifndef TMC2209_PORT_H
#define TMC2209_PORT_H

#include <stdint.h>

typedef struct {
    /*
     * Передача данных по UART.
     * Возврат: 0 при успехе, ненулевой код при ошибке.
     */
    int (*uart_tx)(const uint8_t *data, uint16_t len,
                   uint32_t timeout_ms, void *ctx);

    /*
     * Приём данных по UART.
     * В *received записывается фактически принятое число байт.
     * Возврат: 0 = все байты приняты, 1 = таймаут (частичный приём), <0 = ошибка.
     */
    int (*uart_rx)(uint8_t *data, uint16_t max_len,
                   uint32_t timeout_ms, uint16_t *received, void *ctx);

    /*
     * Сброс приёмного буфера UART: отбросить ожидающие байты, сбросить флаги ошибок.
     */
    void (*uart_rx_flush)(void *ctx);

    /*
     * Задержка в микросекундах (блокирующая).
     */
    void (*delay_us)(uint32_t us, void *ctx);

    /*
     * Управление выводом ENN (enable-not).
     * level=0 — драйвер включён (ENN низкий), level=1 — выключен (ENN высокий).
     */
    void (*set_enable)(uint8_t level, void *ctx);

    /*
     * Опциональный отладочный вывод (может быть NULL).
     * Вызывается с короткими диагностическими строками при инициализации и обмене с регистрами.
     */
    void (*debug_print)(const char *str, void *ctx);

    /*
     * Непрозрачный контекст, передаётся в каждый колбэк.
     * Обычно указывает на платформо-зависимую структуру с хэндлом UART, выводами GPIO и т.д.
     */
    void *ctx;

} tmc2209_io_t;

#endif /* TMC2209_PORT_H */
