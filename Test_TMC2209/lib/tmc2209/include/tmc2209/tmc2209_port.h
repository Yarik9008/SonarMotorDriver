/* tmc2209_port.h — Platform abstraction layer for TMC2209 library.
 *
 * Implement these callbacks for your platform and pass them
 * via tmc2209_io_t to tmc2209_init(). */

#ifndef TMC2209_PORT_H
#define TMC2209_PORT_H

#include <stdint.h>

typedef struct {
    /*
     * Transmit data over UART.
     * Returns: 0 on success, non-zero on error.
     */
    int (*uart_tx)(const uint8_t *data, uint16_t len,
                   uint32_t timeout_ms, void *ctx);

    /*
     * Receive data from UART.
     * Writes number of bytes actually received to *received.
     * Returns: 0 = all bytes received, 1 = timeout (partial), <0 = error.
     */
    int (*uart_rx)(uint8_t *data, uint16_t max_len,
                   uint32_t timeout_ms, uint16_t *received, void *ctx);

    /*
     * Flush UART RX buffer: discard pending bytes, clear error flags.
     */
    void (*uart_rx_flush)(void *ctx);

    /*
     * Microsecond delay (blocking).
     */
    void (*delay_us)(uint32_t us, void *ctx);

    /*
     * Control the ENN (enable-not) pin.
     * level=0 → driver enabled (ENN low), level=1 → disabled (ENN high).
     */
    void (*set_enable)(uint8_t level, void *ctx);

    /*
     * Optional debug/trace output (may be NULL).
     * Called with short diagnostic strings during init and register I/O.
     */
    void (*debug_print)(const char *str, void *ctx);

    /*
     * Opaque context pointer passed to every callback.
     * Typically points to a platform-specific struct holding UART handle,
     * GPIO references, etc.
     */
    void *ctx;

} tmc2209_io_t;

#endif /* TMC2209_PORT_H */
