/* board.h — Аппаратная конфигурация платы (STM32F103C8 + THVD1452 + LENZ IRS). */

#ifndef BOARD_H
#define BOARD_H

#include "stm32f1xx_hal.h"

/* -------- RCC — тактирование (HSI/2 × PLL12 = 48 МГц) -------- */
#define SYSCLK_HZ               48000000U
#define APB1_CLK_HZ             (SYSCLK_HZ / 2U)   /* 24 МГц (макс. 36 МГц) */
#define APB2_CLK_HZ             SYSCLK_HZ           /* 48 МГц */
#define TIM2_CLK_HZ             (APB1_CLK_HZ * 2U)  /* 48 МГц (APB1 prescaler > 1 → ×2) */
#define TIM4_CLK_HZ             TIM2_CLK_HZ         /* TIM4 для STEP импульсов */

/* -------- TIM2 — периодический опрос энкодера -------- */
#define POLL_INTERVAL_MS        1U                            /* Период опроса: 1 мс (1 кГц) */
#define POLL_FREQ_HZ            (1000U / POLL_INTERVAL_MS)    /* 1 кГц */

/* -------- GPIO — светодиод (Blue Pill: PC13, active LOW) -------- */
#define LED_PORT                GPIOC
#define LED_PIN                 GPIO_PIN_13
#define LED_TOGGLE_INTERVAL     (POLL_FREQ_HZ / 2U)  /* Heartbeat ~1 Гц */

/* -------- SYNC — SYNC_OUT / SYNC_IN -------- */
#define SYNC_OUT_PORT           GPIOB
#define SYNC_OUT_PIN            GPIO_PIN_9   /* PB9 — SYNC_OUT (HIGH=позиция достигнута) */
#define SYNC_IN_PORT            GPIOB
#define SYNC_IN_PIN             GPIO_PIN_12  /* PB12 — SYNC_IN (внешний триггер) */
/* Обратная совместимость */
#define SYNC_PORT               SYNC_OUT_PORT
#define SYNC_PIN                SYNC_OUT_PIN

/* -------- SPI1 + THVD1452 — энкодер LENZ IRS (BiSS C) -------- */
#define ENCODER_RESOLUTION_BITS 17U     /* 17 для IRS-I34/I50/I60; 18 для I70/I80/I90 */
#define ENCODER_ACCURACY_DEG    0.05f   /* Погрешность, град (IRS-I50: 0.05, I60: 0.042, I34: 0.15) */
#define ENCODER_STARTUP_MS      50U     /* Время готовности после подачи питания */
#define ENCODER_COUNTS_REV      131072U /* 2^17 для 17-бит энкодера */
#define ENCODER_FAIL_MS         500U    /* Мс без ответа → переход в open-loop */
#define XCVR_DE_PORT            GPIOB
#define XCVR_DE_PIN             GPIO_PIN_0  /* Driver Enable  (active HIGH → TX MA) */
#define XCVR_RE_PORT            GPIOB
#define XCVR_RE_PIN             GPIO_PIN_1  /* Receiver Enable (active LOW → RX SLO) */

/* -------- Режим управления двигателем (compile-time) -------- */
#define MOTOR_DRIVER_MODE_STEP_DIR_VAL  0   /* STEP/DIR импульсы от MCU (TIM4) */
#define MOTOR_DRIVER_MODE_UART_VAL      1   /* VACTUAL / internal pulse generator TMC2209 */
#define MOTOR_DRIVER_MODE               MOTOR_DRIVER_MODE_STEP_DIR_VAL

/* -------- TIM4 + TMC2209 — шаговый двигатель -------- */
#define MOTOR_FULL_STEPS_REV    200U    /* Полных шагов на оборот (200 = 1.8°, 400 = 0.9°) */
#define TMC2209_MICROSTEPS      32U     /* Микрошаг: 1, 2, 4, 8, 16, 32, 64, 128, 256 */
#define MOTOR_STEPS_PER_REV     (MOTOR_FULL_STEPS_REV * TMC2209_MICROSTEPS)
#define MOTOR_DIR_INVERT        1       /* 1 = инвертировать направление */
#define MAX_SPEED_DEG_S         1200U   /* Максимальная скорость, град/с */
#define MAX_STEPS_PER_POLL      ((MAX_SPEED_DEG_S * MOTOR_STEPS_PER_REV + 360U * POLL_FREQ_HZ - 1U) / (360U * POLL_FREQ_HZ))
#define STEP_PORT               GPIOB
#define STEP_PIN                GPIO_PIN_8
#define STEP_PULSE_US           2U      /* Длительность импульса STEP (TMC2209: мин. 1–2 мкс) */
#define DIR_PORT                GPIOB
#define DIR_PIN                 GPIO_PIN_7
#define ENABLE_PORT             GPIOB
#define ENABLE_PIN              GPIO_PIN_6  /* ENN: LOW = включён, HIGH = выключен */

/* -------- USART2 — UART TMC2209 (PDN_UART, single-wire half-duplex) -------- */
#define TMC2209_UART            USART2
#define TMC2209_UART_BAUDRATE    115200U
#define TMC2209_UART_TX_PORT    GPIOA
#define TMC2209_UART_TX_PIN     GPIO_PIN_2   /* PA2 — USART2_TX → PDN_UART (через 1 кОм) */
#define TMC2209_UART_RX_PORT    GPIOA
#define TMC2209_UART_RX_PIN     GPIO_PIN_3   /* PA3 — USART2_RX → PDN_UART (общая линия) */
#define TMC2209_UART_ADDR       0U            /* Адрес драйвера (MS1=0, MS2=0) */
#define TMC2209_RSENSE_OHM      0.11f         /* Резистор измерения тока, Ом */
#define TMC2209_IRUN_MA         800U          /* Ток при движении, мА (IRUN) */
#define TMC2209_IHOLD_MA        400U          /* Ток удержания, мА (IHOLD) */
#define TMC2209_REPLY_DELAY_US  500U          /* Задержка TX→RX для TMC2209 UART */
#define TMC2209_CFG_SENDDELAY   4U            /* SLAVECONF SENDDELAY (0..15) */
#define TMC2209_TPOWERDOWN      20U           /* Задержка снижения тока (0..255) */
#define TMC2209_HALF_DUPLEX     0U            /* 0=full-duplex (TX+1К / RX), 1=half-duplex */
#define TMC2209_SPREADCYCLE     0U            /* 0=StealthChop, 1=SpreadCycle */

/* -------- Старт — целевая позиция при первом чтении энкодера -------- */
#define STARTUP_TARGET_OFFSET_DEG 0.0f  /* Офсет от 0° (0 = идти в 0 по энкодеру) */

/* -------- PID — регулятор -------- */
#define PID_KP_DEFAULT          0.025f
#define PID_KI_DEFAULT          0.0f
#define PID_KD_DEFAULT          0.0f
#define PID_DEADBAND_DEG        0.1f   /* >= ENCODER_ACCURACY_DEG, иначе дребезг */

/* -------- USART1 — UART команд и телеметрии (PA9 TX / PA10 RX, IT) — взаимодействие с ПК -------- */
#define UART_INSTANCE           USART1
#define UART_BAUDRATE           115200U
#define UART_TX_PORT            GPIOA
#define UART_TX_PIN             GPIO_PIN_9    /* PA9  — USART1_TX → ПК (разъём J2 Pin 4) */
#define UART_RX_PORT            GPIOA
#define UART_RX_PIN             GPIO_PIN_10   /* PA10 — USART1_RX ← ПК (разъём J2 Pin 3) */
#define UART_TX_RING_SIZE       512U
#define UART_RX_RING_SIZE       128U

/* -------- USB CDC — виртуальный COM-порт -------- */
#define USB_ENUM_DELAY_MS       1500U   /* Пауза для энумерации хостом */
#define USB_TX_RING_SIZE        1024U   /* Размер кольцевого буфера передачи */
#define USB_RX_RING_SIZE        256U    /* Размер кольцевого буфера приёма */

/* -------- DWT-задержки (блокирующие, только HAL/USB internals) -------- */
void Delay_Init(void);
void Delay_ms(uint32_t ms);

/* -------- IWDG — сторожевой таймер -------- */
#define IWDG_PRESCALER          IWDG_PRESCALER_64   /* LSI 40 кГц / 64 = 625 Гц */
#define IWDG_RELOAD             312U                /* 312 / 625 ≈ 500 мс таймаут */

/* -------- NVIC — приоритеты прерываний (меньше = выше) -------- */
#define IRQ_PRIO_USB            5U
#define IRQ_PRIO_UART           6U
#define IRQ_PRIO_TIM_POLL       6U
#define IRQ_PRIO_STEP           7U

/* -------- Телеметрия -------- */
#define OUTPUT_PERIOD_MS_DEFAULT 4U    /* Период (мс), 0 = отключить; 4 мс = 250 Гц */
#define TELEMETRY_DEBUG_DEFAULT 0       /* 0 = cp,ec; 1 = полная телеметрия */

/* Состав телеметрии:
 * debug=0 (обычная): cp(float), ec(uint8_t)
 * debug=1 (полная):  cp(float), tp(float), pe(float), u(float), m("cl"|"ol"), ec(uint8_t), kp(float), ki(float), kd(float) 
 * */

/* -------- Коды ошибок телеметрии -------- */
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
