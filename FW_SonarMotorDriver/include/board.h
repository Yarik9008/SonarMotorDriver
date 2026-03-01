/* board.h — Аппаратная конфигурация платы (STM32F103C8 + THVD1452 + LENZ IRS). */

#ifndef BOARD_H
#define BOARD_H

#include "stm32f1xx_hal.h"

/* -------- Дерево тактирования (HSI/2 × PLL12 = 48 МГц) -------- */
#define SYSCLK_HZ               48000000U
#define APB1_CLK_HZ             (SYSCLK_HZ / 2U)   /* 24 МГц (макс. 36 МГц) */
#define APB2_CLK_HZ             SYSCLK_HZ           /* 48 МГц */
#define TIM2_CLK_HZ             (APB1_CLK_HZ * 2U)  /* 48 МГц (APB1 prescaler > 1 → ×2) */
#define TIM4_CLK_HZ             TIM2_CLK_HZ        /* TIM4 для STEP импульсов */

/* -------- Периодический опрос энкодера (TIM2) -------- */
#define POLL_INTERVAL_MS        1U                            /* Период опроса: 1 мс (1 кГц) */
#define POLL_FREQ_HZ            (1000U / POLL_INTERVAL_MS)    /* 1 кГц */

/* -------- Светодиод на плате (Blue Pill: PC13, active LOW) -------- */
#define LED_PORT                GPIOC
#define LED_PIN                 GPIO_PIN_13
#define LED_TOGGLE_INTERVAL     (POLL_FREQ_HZ / 2U)  /* Heartbeat ~1 Гц */

/* -------- Энкодер LENZ IRS -------- */
#define ENCODER_RESOLUTION_BITS 17U     /* 17 для IRS-I34/I50/I60; 18 для I70/I80/I90 */
#define ENCODER_ACCURACY_DEG    0.05f   /* Погрешность энкодера, град (IRS-I50: 0.05, I60: 0.042, I34: 0.15) */
#define ENCODER_STARTUP_MS      50U     /* Время готовности после подачи питания */
#define ENCODER_COUNTS_REV      131072U /* 2^17 для 17-бит энкодера */
#define ENCODER_FAIL_MS         500U    /* Мс без ответа → переход в open-loop */

/* -------- Шаговый двигатель -------- */
#define MOTOR_FULL_STEPS_REV    200U    /* Полных шагов на оборот (200 = 1.8°, 400 = 0.9°) */
#define TMC2208_MICROSTEPS      32U     /* Микрошаг: 1, 2, 4, 8, 16, 32, 64, 128, 256 */
#define MOTOR_STEPS_PER_REV     (MOTOR_FULL_STEPS_REV * TMC2208_MICROSTEPS) /* Микрошагов на оборот */
#define MOTOR_DIR_INVERT        1       /* 1 = инвертировать направление шагов (если мотор крутит не туда) */
#define MAX_SPEED_DEG_S         1000U   /* Максимальная скорость, град/с */
#define MAX_STEPS_PER_POLL      ((MAX_SPEED_DEG_S * MOTOR_STEPS_PER_REV + 360U * POLL_FREQ_HZ - 1U) / (360U * POLL_FREQ_HZ))

/* -------- PID по умолчанию -------- */
#define PID_KP_DEFAULT          0.01f
#define PID_KI_DEFAULT          0.0f
#define PID_KD_DEFAULT          0.0f
#define PID_DEADBAND_DEG        0.05f

/* -------- RS-422 трансивер THVD1452 -------- */
#define XCVR_DE_PORT            GPIOB
#define XCVR_DE_PIN             GPIO_PIN_0  /* Driver Enable  (active HIGH → TX MA) */
#define XCVR_RE_PORT            GPIOB
#define XCVR_RE_PIN             GPIO_PIN_1  /* Receiver Enable (active LOW → RX SLO) */

/* -------- TMC2208 шаговый драйвер (STEP/DIR/ENABLE) -------- */
#define STEP_PORT               GPIOB
#define STEP_PIN                GPIO_PIN_8
#define STEP_PULSE_US           2U     /* Длительность импульса STEP (TMC2208: мин. 1–2 мкс) */
#define DIR_PORT                GPIOB
#define DIR_PIN                 GPIO_PIN_7
#define ENABLE_PORT             GPIOB
#define ENABLE_PIN              GPIO_PIN_6   /* ENN: LOW = включён, HIGH = выключен */

/* -------- Приоритеты прерываний NVIC (меньше = выше приоритет) -------- */
#define IRQ_PRIO_USB            5U
#define IRQ_PRIO_TIM_POLL       6U
#define IRQ_PRIO_STEP           7U

/* -------- Периодический вывод cp/tp/pe/m/ec -------- */
#define OUTPUT_PERIOD_MS_DEFAULT 5U     /* Период по умолчанию (мс), 0 = отключить; 5 мс = 200 Гц */

/* -------- USB -------- */
#define USB_ENUM_DELAY_MS       1500U   /* Пауза для энумерации хостом */
#define USB_TX_RING_SIZE        1024U   /* Размер кольцевого буфера передачи */
#define USB_RX_RING_SIZE        256U    /* Размер кольцевого буфера приёма */

/* -------- Общий код ошибки телеметрии (err) -------- */
typedef enum {
    ERR_OK = 0,
    ERR_BISS_CRC,
    ERR_BISS_NO_RESP,
    ERR_BISS_SENSOR,
    ERR_BISS_WARN,
    ERR_BISS_SPI,
    ERR_COUNT
} ErrCode;

#endif /* BOARD_H */
