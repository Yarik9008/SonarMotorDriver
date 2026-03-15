/**
 * @file tmc2209_port_stm32_hal.h
 * @brief STM32 HAL platform adapter for TMC2209 library.
 */

#ifndef TMC2209_PORT_STM32_HAL_H
#define TMC2209_PORT_STM32_HAL_H

#include "tmc2209_port.h"
#include "stm32f1xx_hal.h"

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

/* Fill tmc2209_io_t with STM32 HAL callback implementations. */
void tmc2209_port_stm32_hal_fill_io(tmc2209_io_t *io, tmc2209_hal_ctx_t *hal);

/* Initialize DWT cycle counter. */
void tmc2209_port_stm32_dwt_init(void);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_PORT_STM32_HAL_H */
