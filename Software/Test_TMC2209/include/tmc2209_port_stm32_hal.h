/* tmc2209_port_stm32_hal.h — адаптер под STM32 HAL для библиотеки TMC2209.
 *
 * Реализует колбэки tmc2209_io_t на базе STM32 HAL (UART, GPIO, счётчик циклов DWT).
 */

#ifndef TMC2209_PORT_STM32_HAL_H
#define TMC2209_PORT_STM32_HAL_H

#include "tmc2209/tmc2209_port.h"
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

/* Заполняет tmc2209_io_t реализациями колбэков на STM32 HAL.
 * Указатель hal_ctx должен оставаться действительным на всё время работы драйвера. */
void tmc2209_port_stm32_hal_fill_io(tmc2209_io_t *io, tmc2209_hal_ctx_t *hal);

/* Инициализация счётчика циклов DWT (вызвать один раз при старте до использования драйвера). */
void tmc2209_port_stm32_dwt_init(void);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_PORT_STM32_HAL_H */
