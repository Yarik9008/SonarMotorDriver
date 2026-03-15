/* tmc2209_port_stm32_hal.h — HAL-адаптер для lib/tmc2209.
 *
 * Предоставляет реализации колбэков tmc2209_io_t на основе STM32 HAL
 * (UART, GPIO, DWT счётчик циклов).
 */

#ifndef TMC2209_PORT_STM32_HAL_H
#define TMC2209_PORT_STM32_HAL_H

#include "tmc2209/tmc2209_port.h"
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Контекст HAL-порта: UART, пины, системная частота, режим half-duplex. */
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *en_port;
    uint16_t            en_pin;
    uint32_t            sysclk_hz;
    uint8_t             half_duplex;
    void (*debug_fn)(const char *str);
} tmc2209_hal_ctx_t;

/* Заполняет tmc2209_io_t реализациями колбэков. hal_ctx должен быть валиден весь срок жизни драйвера. */
void tmc2209_port_stm32_hal_fill_io(tmc2209_io_t *io, tmc2209_hal_ctx_t *hal);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_PORT_STM32_HAL_H */
