/**
 * @file board.h
 * @brief Аппаратная конфигурация платы (STM32F103C8 Blue Pill + THVD1452 + LENZ IRS).
 *
 * Централизованное определение выводов, частот тактирования, параметров
 * опроса и приоритетов прерываний. При смене аппаратной конфигурации
 * или переносе на другую плату достаточно изменить этот файл.
 */

#ifndef BOARD_H
#define BOARD_H

#include "stm32f1xx_hal.h"

/* -------- Дерево тактирования (HSI/2 × PLL12 = 48 МГц) -------- */
#define SYSCLK_HZ               48000000U
#define APB1_CLK_HZ             (SYSCLK_HZ / 2U)   /* 24 МГц (макс. 36 МГц) */
#define APB2_CLK_HZ             SYSCLK_HZ           /* 48 МГц */
#define TIM2_CLK_HZ             (APB1_CLK_HZ * 2U)  /* 48 МГц (APB1 prescaler > 1 → ×2) */

/* -------- Периодический опрос энкодера (TIM2) -------- */
#define POLL_INTERVAL_MS        2U                            /* Период опроса: 2 мс */
#define POLL_FREQ_HZ            (1000U / POLL_INTERVAL_MS)    /* 500 Гц */

/* -------- Светодиод на плате (Blue Pill: PC13, active LOW) -------- */
#define LED_PORT                GPIOC
#define LED_PIN                 GPIO_PIN_13
#define LED_TOGGLE_INTERVAL     (POLL_FREQ_HZ / 2U)  /* Heartbeat ~1 Гц */

/* -------- Энкодер LENZ IRS -------- */
#define ENCODER_RESOLUTION_BITS 17U     /* 17 для IRS-I34/I50/I60; 18 для I70/I80/I90 */
#define ENCODER_STARTUP_MS      50U     /* Время готовности после подачи питания */

/* -------- RS-422 трансивер THVD1452 -------- */
#define XCVR_DE_PORT            GPIOB
#define XCVR_DE_PIN             GPIO_PIN_0  /* Driver Enable  (active HIGH → TX MA) */
#define XCVR_RE_PORT            GPIOB
#define XCVR_RE_PIN             GPIO_PIN_1  /* Receiver Enable (active LOW → RX SLO) */

/* -------- Приоритеты прерываний NVIC (меньше = выше приоритет) -------- */
#define IRQ_PRIO_USB            5U
#define IRQ_PRIO_TIM_POLL       6U

/* -------- USB -------- */
#define USB_ENUM_DELAY_MS       1500U   /* Пауза для энумерации хостом */
#define USB_TX_RING_SIZE        1024U   /* Размер кольцевого буфера передачи */

#endif /* BOARD_H */
