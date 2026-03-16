/**
 * @file tmc2209_port_stm32_hal.h
 * @brief Реализация порта (hardware abstraction layer) для STM32 HAL.
 *
 * Модуль связывает абстрактные колбэки библиотеки tmc2209.h с конкретной
 * периферией STM32: UART, GPIO и таймерами (TIM). Описывает структуру
 * контекста hal_ctx_t.
 */

#ifndef TMC2209_PORT_STM32_HAL_H
#define TMC2209_PORT_STM32_HAL_H

#include "tmc2209_port.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Контекст STM32 HAL для TMC2209.
 * Включает UART для обмена с чипом и TIM/GPIO для бэкенда мотора (STEP/DIR).
 */
typedef struct {
    /* UART */
    UART_HandleTypeDef *huart;
    uint8_t             half_duplex;
    uint32_t            sysclk_hz;

    /* Вывод разрешения (EN) */
    GPIO_TypeDef       *en_port;
    uint16_t            en_pin;

    /* Бэкенд мотора (STEP/DIR) */
    TIM_HandleTypeDef  *htim_step;
    uint32_t            tim_channel;
    GPIO_TypeDef       *step_port;
    uint16_t            step_pin;
    GPIO_TypeDef       *dir_port;
    uint16_t            dir_pin;

    /* Прочее */
    void (*debug_fn)(const char *str);
} tmc2209_hal_ctx_t;

/** Заполнить колбэки ввода-вывода реализациями на STM32 HAL. */
void tmc2209_port_stm32_hal_fill_io(tmc2209_io_t *io, tmc2209_hal_ctx_t *hal);

/* Вспомогательные функции HAL MSP для UART TMC2209 (USART2). */
void tmc2209_port_stm32_hal_uart_msp_init(UART_HandleTypeDef *h);
void tmc2209_port_stm32_hal_uart_msp_deinit(UART_HandleTypeDef *h);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_PORT_STM32_HAL_H */
