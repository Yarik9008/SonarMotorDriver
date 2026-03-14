/* board.h — Board configuration for Test_TMC2209 (STM32F446RE + USB CDC).
 *
 * Defines hardware pin assignments, clock config, and board-specific
 * parameters. Used by application code (main.c, usbd_conf.c), NOT by
 * the TMC2209 library (which is platform-agnostic).
 */

#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx_hal.h"

/* -------- RCC -------- */
#define SYSCLK_HZ               168000000U

/* -------- USB CDC -------- */
#define USB_TX_RING_SIZE        512U
#define USB_RX_RING_SIZE        256U
#define IRQ_PRIO_USB            5U

/* -------- USART2 — TMC2209 PDN_UART (hardware wiring) -------- */
#define TMC2209_UART            USART2
#define TMC2209_UART_BAUDRATE   115200U
#define TMC2209_UART_TX_PORT    GPIOA
#define TMC2209_UART_TX_PIN     GPIO_PIN_2   /* PA2 — TX */
#define TMC2209_UART_RX_PORT    GPIOA
#define TMC2209_UART_RX_PIN     GPIO_PIN_3   /* PA3 — RX */
#define IRQ_PRIO_TMC_UART       6U

/* -------- TMC2209 motor parameters (used by app to fill config) -------- */
#define TMC2209_UART_ADDR       0U
#define TMC_REPLY_DELAY_US      500U
#define TMC2209_RSENSE_OHM      0.11f
#define TMC2209_IRUN_MA         800U
#define TMC2209_IHOLD_MA        400U
#define TMC2209_MICROSTEPS      16U

/* -------- GPIO — TMC2209 STEP/DIR/ENN -------- */
#define STEP_PORT               GPIOB
#define STEP_PIN                GPIO_PIN_8   /* PB8 — STEP (TIM4_CH3 AF2) */
#define DIR_PORT                GPIOB
#define DIR_PIN                 GPIO_PIN_7   /* PB7 — DIR */
#define ENABLE_PORT             GPIOB
#define ENABLE_PIN              GPIO_PIN_6   /* PB6 — ENN: LOW=on, HIGH=off */

/* -------- TIM4 — STEP pulse generator (PWM CH3 on PB8) -------- */
#define STEP_TIM                TIM4
#define STEP_TIM_CH             TIM_CHANNEL_3
#define STEP_TIM_AF             GPIO_AF2_TIM4
#define STEP_TIM_CLK_HZ        84000000U    /* APB1 timer clock */
#define STEP_TIM_PSC            83U          /* 84 MHz / 84 = 1 MHz counter */
#define STEP_TIM_PULSE_US       2U
#define IRQ_PRIO_STEP           7U

/* -------- Utilities -------- */
static inline void Delay_ms(uint32_t ms) { HAL_Delay(ms); }

#endif /* BOARD_H */
