/* board.h — конфигурация платы Test_TMC2209 (STM32F103C8).
 *
 * Задаёт назначение выводов, настройки тактирования и параметры платы.
 * Используется приложением (main.c). Библиотека TMC2209 не привязана к платформе.
 */

#ifndef BOARD_H
#define BOARD_H

#include "stm32f1xx_hal.h"

/* -------- RCC (тактовые частоты) -------- */
#define SYSCLK_HZ               48000000U
#define APB1_CLK_HZ             (SYSCLK_HZ / 2U)
#define TIM4_CLK_HZ             SYSCLK_HZ

/* -------- USART1 — CLI: приём команд и ответы (PA9=TX, PA10=RX), 115200 8N1 -------- */
#define CLI_UART                USART1
#define CLI_UART_BAUDRATE      115200U
#define CLI_UART_TX_PORT       GPIOA
#define CLI_UART_TX_PIN        GPIO_PIN_9   /* PA9 — TX (ответы в терминал) */
#define CLI_UART_RX_PORT       GPIOA
#define CLI_UART_RX_PIN        GPIO_PIN_10  /* PA10 — RX (команды с ПК) */
#define IRQ_PRIO_CLI_UART      6U

/* -------- Задержка после включения питания TMC2209 (мс) перед первой инициализацией UART -------- */
#define TMC2209_POWERON_DELAY_MS    100U
#define TMC2209_INIT_RETRY_DELAY_MS 200U

/* -------- USART2 — линия PDN_UART для обмена с TMC2209 -------- */
#define TMC2209_UART            USART2
#define TMC2209_UART_BAUDRATE   115200U
#define TMC2209_UART_TX_PORT    GPIOA
#define TMC2209_UART_TX_PIN     GPIO_PIN_2
#define TMC2209_UART_RX_PORT    GPIOA
#define TMC2209_UART_RX_PIN     GPIO_PIN_3
#define IRQ_PRIO_TMC_UART       6U

/* -------- Параметры двигателя и драйвера TMC2209 -------- */
/* Внимание: линию питания драйвера VS необходимо ВСЕГДА подключать через дополнительный электролитический конденсатор! */
#define TMC2209_UART_ADDR       0U
#define TMC_REPLY_DELAY_US      500U
#define TMC2209_RSENSE_OHM      0.11f
#define TMC2209_IRUN_MA         800U
#define TMC2209_IHOLD_MA        400U
#define TMC2209_MICROSTEPS      16U

/* -------- GPIO — выводы TMC2209: STEP, DIR, ENN -------- */
#define STEP_PORT               GPIOB
#define STEP_PIN                GPIO_PIN_8
#define DIR_PORT                GPIOB
#define DIR_PIN                 GPIO_PIN_7
#define ENABLE_PORT             GPIOB
#define ENABLE_PIN              GPIO_PIN_6   /* ENN: LOW — драйвер включён, HIGH — выключен */

/* -------- TIM4 — генератор импульсов STEP (ШИМ, канал 3 на PB8) -------- */
#define STEP_TIM                TIM4
#define STEP_TIM_CH             TIM_CHANNEL_3
#define STEP_TIM_CLK_HZ        48000000U    /* такт. таймера: APB1 * 2 при предделителе APB1 > 1 */
#define STEP_TIM_PSC            47U          /* 48 МГц / 48 = 1 МГц счётчик */
#define STEP_TIM_PULSE_US       2U
#define IRQ_PRIO_STEP           7U

/* -------- Вспомогательные макросы -------- */
static inline void Delay_ms(uint32_t ms) { HAL_Delay(ms); }

#endif /* BOARD_H */
