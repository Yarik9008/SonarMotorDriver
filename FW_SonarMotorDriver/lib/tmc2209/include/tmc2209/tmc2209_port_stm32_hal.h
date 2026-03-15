/* tmc2209_port_stm32_hal.h — STM32 HAL port for lib/tmc2209. */

#ifndef TMC2209_PORT_STM32_HAL_H
#define TMC2209_PORT_STM32_HAL_H

#include "tmc2209_port.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* STM32 HAL Context for TMC2209. 
 * Includes UART for chip communication and TIM/GPIO for motor backend. 
 */
typedef struct {
    /* UART */
    UART_HandleTypeDef *huart;
    uint8_t             half_duplex;
    uint32_t            sysclk_hz;

    /* Enable Pin */
    GPIO_TypeDef       *en_port;
    uint16_t            en_pin;

    /* Motor Backend (STEP/DIR) */
    TIM_HandleTypeDef  *htim_step;
    uint32_t            tim_channel;
    GPIO_TypeDef       *step_port;
    uint16_t            step_pin;
    GPIO_TypeDef       *dir_port;
    uint16_t            dir_pin;

    /* Misc */
    void (*debug_fn)(const char *str);
} tmc2209_hal_ctx_t;

/** Fill i/o callbacks with STM32 HAL implementations. */
void tmc2209_port_stm32_hal_fill_io(tmc2209_io_t *io, tmc2209_hal_ctx_t *hal);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_PORT_STM32_HAL_H */
