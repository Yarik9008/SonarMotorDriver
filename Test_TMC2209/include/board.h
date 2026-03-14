/* board.h — конфигурация платы Test_TMC2208 (STM32F446 + USB CDC + TMC2209). */

#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx_hal.h"

/* -------- RCC -------- */
#define SYSCLK_HZ               168000000U

/* -------- USB CDC -------- */
#define USB_TX_RING_SIZE        512U
#define USB_RX_RING_SIZE        256U
#define IRQ_PRIO_USB            5U

/* -------- USART2 — UART TMC2209 (PDN_UART, single-wire half-duplex) -------- */
#define TMC2209_UART            USART2
#define TMC2209_UART_BAUDRATE   115200U
#define TMC2209_UART_TX_PORT    GPIOA
#define TMC2209_UART_TX_PIN     GPIO_PIN_2   /* PA2 — USART2_TX → PDN_UART (через 1 кОм) */
#define TMC2209_UART_RX_PORT    GPIOA
#define TMC2209_UART_RX_PIN     GPIO_PIN_3   /* PA3 — USART2_RX → PDN_UART (общая линия) */
#define TMC2209_UART_ADDR       0U           /* Адрес драйвера (MS1=0, MS2=0) */
#define TMC2209_RSENSE_OHM      0.11f        /* Резистор измерения тока, Ом */
#define TMC2209_IRUN_MA         1000U        /* Ток при движении, мА (IRUN) 1A */
#define TMC2209_IHOLD_MA        500U         /* Ток удержания, мА (IHOLD) 0.5A */
#define TMC2209_MICROSTEPS      128U         /* Микрошаг: 1, 2, 4, 8, 16, 32, 64, 128, 256 */
#define TMC2209_USE_UART_MODE   1U           /* 1 = UART-режим (VACTUAL) по умолчанию */
#define IRQ_PRIO_TMC_UART       6U

/* -------- GPIO — TMC2209 STEP/DIR/ENN -------- */
#define STEP_PORT               GPIOB
#define STEP_PIN                GPIO_PIN_8   /* PB8 — STEP (TIM4_CH3 AF2) */
#define DIR_PORT                GPIOB
#define DIR_PIN                 GPIO_PIN_7   /* PB7 — DIR */
#define ENABLE_PORT             GPIOB
#define ENABLE_PIN              GPIO_PIN_6   /* PB6 — ENN: LOW = вкл, HIGH = выкл */

/* -------- TIM4 — генератор STEP-импульсов (PWM CH3 на PB8) -------- */
#define STEP_TIM                TIM4
#define STEP_TIM_CH             TIM_CHANNEL_3
#define STEP_TIM_AF             GPIO_AF2_TIM4
#define STEP_TIM_CLK_HZ        84000000U   /* APB1 timer clock (HCLK/4*2) */
#define STEP_TIM_PSC            83U         /* 84 MHz / 84 = 1 MHz counter */
#define STEP_TIM_PULSE_US       2U
#define IRQ_PRIO_STEP           7U

/* -------- Утилиты -------- */
static inline void Delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

#endif /* BOARD_H */
