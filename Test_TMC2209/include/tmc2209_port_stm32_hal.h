/* tmc2209_port_stm32_hal.h — STM32 HAL platform adapter for TMC2209 library.
 *
 * Provides tmc2209_io_t callback implementations using STM32 HAL
 * (UART, GPIO, DWT cycle counter).
 */

#ifndef TMC2209_PORT_STM32_HAL_H
#define TMC2209_PORT_STM32_HAL_H

#include "tmc2209/tmc2209_port.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *en_port;
    uint16_t            en_pin;
    uint32_t            sysclk_hz;
    uint8_t             half_duplex;
    void (*debug_fn)(const char *str);
} tmc2209_hal_ctx_t;

/* Fill tmc2209_io_t with STM32 HAL callback implementations.
 * hal_ctx must remain valid for the lifetime of the driver. */
void tmc2209_port_stm32_hal_fill_io(tmc2209_io_t *io, tmc2209_hal_ctx_t *hal);

/* Initialize DWT cycle counter (call once at startup before using the driver). */
void tmc2209_port_stm32_dwt_init(void);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_PORT_STM32_HAL_H */
