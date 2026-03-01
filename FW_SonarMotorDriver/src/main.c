/* main.c — Позиционное управление шаговым двигателем (PID + BiSS-C энкодер). */

#include "stm32f1xx_hal.h" /* HAL */
#include "board.h"
#include "biss_c.h" /* BiSS C энкодер */
#include "usb_cdc.h" /* USB CDC */
#include "stepper.h" /* Шаговый двигатель */
#include "pid.h" /* PID регулятор */
#include "cmd_parser.h" /* Парсер команд */
#include "uart.h" /* UART */
#include <stdio.h> /* printf */
#include <string.h> /* strlen */    
#include <stdarg.h> /* va_list */

/* --- Прототипы --- */

static void SystemClock_Config(void);
static void BSP_Init(void); /* Инициализация светодиода */
static void PollTimer_Init(void); /* Инициализация таймера 1 кГц */
static void ProcessCommand(const Cmd_Result *cmd); /* Обработка команд */
static void SendResponse(const char *fmt, ...); /* Отправка ответа */
static void PollCommands(void); /* Обработка команд */
static void Encoder_Accumulate(const BiSS_Reading *rd); /* Суммирование энкодера */
static void Mode_Update(uint8_t enc_ok); /* Автопереключение CL ↔ OL */
static void MotorControl_Tick(uint8_t enc_ok); /* Управление мотором */
static void Telemetry_Tick(uint8_t enc_ok, BiSS_Status st); /* Телеметрия */
static void Heartbeat_Tick(void); /* Heartbeat */

/* --- Единицы: всё в градусах --- */

#define DEG_PER_STEP        (360.0f / (float)MOTOR_STEPS_PER_REV)
#define MAX_DEG_PER_TICK    ((float)MAX_SPEED_DEG_S / (float)POLL_FREQ_HZ) /* Максимальная скорость, град/с */  
#define DT_S                (1.0f / (float)POLL_FREQ_HZ) /* Интервал времени, с */
#define COUNTS_TO_DEG(c)    ((float)(c) * 360.0f / (float)ENCODER_COUNTS_REV) /* Конвертация счетчиков в градусы */

static inline int32_t DegToSteps(float deg) /* Конвертация градусов в шаги */
{
    return (int32_t)(deg / DEG_PER_STEP);
}

static inline float clampf(float v, float lo, float hi) /* Ограничение значения */
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) /* Ограничение значения */  
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Нормализация ошибки в (-180, 180] — кратчайший путь к цели */
static inline float shortest_path_err(float target, float pos)
{
    float err = target - pos;
    while (err > 180.0f)  err -= 360.0f;
    while (err <= -180.0f) err += 360.0f;
    return err;
}

/* --- Периферия --- */

static TIM_HandleTypeDef htim_poll; /* Таймер 1 кГц */
static IWDG_HandleTypeDef hiwdg; /* Сторожевой таймер */

/* --- Состояние --- */

typedef enum { MODE_CLOSED_LOOP = 0, MODE_OPEN_LOOP = 1 } CtrlMode; /* Режим управления */

static PID_State g_pid = { /* PID регулятор */
    .kp = PID_KP_DEFAULT, 
    .ki = PID_KI_DEFAULT, 
    .kd = PID_KD_DEFAULT,
    .integral = 0.0f, 
    .prev_error = 0.0f,
    .output_min = -MAX_DEG_PER_TICK,
    .output_max =  MAX_DEG_PER_TICK,
    .initialized = 0
};

static float    g_target_deg       = 0.0f; /* Целевая позиция, градусы */
static uint8_t  g_enabled          = 0;
static uint16_t g_output_period_ms = OUTPUT_PERIOD_MS_DEFAULT; /* Период вывода телеметрии, мс */
static uint8_t  g_telemetry_debug = TELEMETRY_DEBUG_DEFAULT;  /* 0 = cp,ec; 1 = полная */

static uint32_t g_enc_raw_prev     = 0xFFFFFFFF; /* Предыдущее значение энкодера */
static int64_t  g_enc_counts       = 0; /* Сумма энкодера */

static float    g_cl_deg_accum     = 0.0f; /* Сумма интеграла PID */
static float    g_last_ctrl_deg    = 0.0f; /* Последнее управляющее воздействие, град */

static CtrlMode g_mode             = MODE_CLOSED_LOOP;
static uint32_t g_enc_fail_cnt     = 0;
static float    g_ol_pos_deg       = 0.0f; /* Позиция в open-loop, градусы */
static uint8_t  g_was_outside_db   = 0;    /* Флаг: были за пределами deadband (для snap) */
static uint8_t  g_homing           = 0;    /* 1 = хоминг к начальной позиции (при старте) */


/* --- Вспомогательные --- */

static void DoSteps(int32_t steps) /* Выполнение шагов */
{
#if MOTOR_DIR_INVERT /* Инвертированное направление */
    Stepper_Steps(-steps);
#else
    Stepper_Steps(steps); /* Прямое направление */
#endif
}

/* --- MAIN --- */

int main(void)
{
    HAL_Init();
    SystemClock_Config(); /* Инициализация системного таймера */    
    BSP_Init();

    USB_CDC_Init(); /* Инициализация USB CDC */
    UART_Init(); /* Инициализация UART */
    HAL_Delay(USB_ENUM_DELAY_MS); /* Задержка для инициализации USB */

    BiSS_Config enc_cfg = { /* Конфигурация энкодера */
        .spi_instance    = SPI1,
        .resolution_bits = ENCODER_RESOLUTION_BITS,
        .de_port         = XCVR_DE_PORT,
        .de_pin          = XCVR_DE_PIN,
        .re_port         = XCVR_RE_PORT,
        .re_pin          = XCVR_RE_PIN,
    };
    BiSS_Init(&enc_cfg); /* Инициализация энкодера */
    HAL_Delay(ENCODER_STARTUP_MS); /* Задержка для инициализации энкодера */

    Stepper_Init(); /* Инициализация шагового двигателя */

    PollTimer_Init(); /* Инициализация таймера 1 кГц */
    HAL_TIM_Base_Start(&htim_poll); /* Запуск таймера 1 кГц */

    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER;
    hiwdg.Init.Reload    = IWDG_RELOAD;
    HAL_IWDG_Init(&hiwdg);

    /* Первое блокирующее чтение для инициализации позиции */
    BiSS_Reading rd;
    BiSS_Status  st = BiSS_Read(&rd);
    if (st == BISS_OK || st == BISS_ERR_WARNING)
        Encoder_Accumulate(&rd);

    /* Запуск первого DMA-чтения */
    BiSS_StartRead();
    uint8_t dma_pending = 1;
    uint8_t enc_ok      = (g_enc_raw_prev != 0xFFFFFFFF);

    while (1) {
        USB_CDC_Task();
        UART_Task();
        PollCommands();

        /* Ожидание тика 1 кГц */
        if (!__HAL_TIM_GET_FLAG(&htim_poll, TIM_FLAG_UPDATE))
            continue;
        __HAL_TIM_CLEAR_FLAG(&htim_poll, TIM_FLAG_UPDATE);

        /* 1. Забрать результат предыдущего DMA-чтения */
        if (dma_pending && BiSS_IsReady()) {
            st = BiSS_GetResult(&rd);
            enc_ok = (st == BISS_OK || st == BISS_ERR_WARNING);
            if (enc_ok)
                Encoder_Accumulate(&rd);
            dma_pending = 0;
        }

        /* 2. Запуск следующего DMA-чтения (пока CPU обрабатывает PID) */
        if (!dma_pending) {
            BiSS_StartRead();
            dma_pending = 1;
        }

        /* 3. Контур управления */
        Mode_Update(enc_ok);
        MotorControl_Tick(enc_ok);
        Telemetry_Tick(enc_ok, st);
        Heartbeat_Tick();

        HAL_IWDG_Refresh(&hiwdg);
    }
}

/* --- Многооборотная позиция --- */

static void Encoder_Accumulate(const BiSS_Reading *rd)
{
    if (g_enc_raw_prev == 0xFFFFFFFF) {
        g_enc_counts   = (int64_t)rd->position;
        g_enc_raw_prev = rd->position;
        float pos_deg  = COUNTS_TO_DEG(g_enc_counts);
        float err      = shortest_path_err(STARTUP_TARGET_OFFSET_DEG, pos_deg);
        g_target_deg   = pos_deg + err;   /* Цель в многооборотных координатах = кратчайший путь */
        g_ol_pos_deg   = pos_deg;
        g_homing       = 1;
        g_enabled      = 1;
        Stepper_SetEnable(1);
        return;
    }

    int32_t delta = (int32_t)rd->position - (int32_t)g_enc_raw_prev;
    if (delta > (int32_t)(ENCODER_COUNTS_REV / 2))
        delta -= (int32_t)ENCODER_COUNTS_REV;
    else if (delta < -(int32_t)(ENCODER_COUNTS_REV / 2))
        delta += (int32_t)ENCODER_COUNTS_REV;

    g_enc_counts   += delta;
    g_enc_raw_prev = rd->position;
}

/* --- Автопереключение CL ↔ OL --- */

static void Mode_Update(uint8_t enc_ok)
{
    if (enc_ok) {
        g_enc_fail_cnt = 0;
        if (g_mode == MODE_OPEN_LOOP) {
            g_mode = MODE_CLOSED_LOOP;
            PID_Reset(&g_pid);
            g_cl_deg_accum = 0.0f;
        }
    } else {
        g_enc_fail_cnt++;
        if (g_mode == MODE_CLOSED_LOOP &&
            g_enc_fail_cnt >= (ENCODER_FAIL_MS / POLL_INTERVAL_MS))
        {
            g_mode       = MODE_OPEN_LOOP;
            g_ol_pos_deg = COUNTS_TO_DEG(g_enc_counts);
        }
    }
}

/* --- Управление мотором --- */

static void MotorControl_Tick(uint8_t enc_ok)
{
    if (!g_enabled) {
        Stepper_Stop();
        g_last_ctrl_deg = 0.0f;
        return;
    }

    g_last_ctrl_deg = 0.0f;

    if (g_mode == MODE_CLOSED_LOOP && enc_ok) {
        float pos_deg = COUNTS_TO_DEG(g_enc_counts);
        float err_deg = g_target_deg - pos_deg;

        if (err_deg > PID_DEADBAND_DEG || err_deg < -PID_DEADBAND_DEG) {
            g_was_outside_db = 1;
            float pid_out = PID_Update(&g_pid, err_deg, DT_S);
            g_cl_deg_accum = clampf(g_cl_deg_accum + pid_out,
                                    -MAX_DEG_PER_TICK, MAX_DEG_PER_TICK);

            int32_t steps = clampi(DegToSteps(g_cl_deg_accum),
                                   -(int32_t)MAX_STEPS_PER_POLL,
                                    (int32_t)MAX_STEPS_PER_POLL);
            if (steps != 0) {
                DoSteps(steps);
                g_cl_deg_accum -= (float)steps * DEG_PER_STEP;
                g_last_ctrl_deg = (float)steps * DEG_PER_STEP;
            } else {
                g_last_ctrl_deg = pid_out;
            }
        } else {
            if (g_was_outside_db) {
                if (err_deg > 0.02f || err_deg < -0.02f) {
                    int32_t snap = (err_deg > 0.0f) ? 1 : -1;
                    DoSteps(snap);
                    g_last_ctrl_deg = (float)snap * DEG_PER_STEP;
                }
                g_was_outside_db = 0;
            }
            if (g_homing) {
                g_homing = 0;
                g_enc_counts = (int64_t)g_enc_raw_prev;
                g_target_deg = STARTUP_TARGET_OFFSET_DEG;
                float pos = COUNTS_TO_DEG(g_enc_counts);
                if (pos - g_target_deg > 180.0f)
                    g_enc_counts -= (int64_t)ENCODER_COUNTS_REV;
                else if (pos - g_target_deg < -180.0f)
                    g_enc_counts += (int64_t)ENCODER_COUNTS_REV;
            }
            Stepper_Stop();
            PID_Reset(&g_pid);
            g_cl_deg_accum = 0.0f;
        }

    } else if (g_mode == MODE_OPEN_LOOP) {
        float err_deg = g_target_deg - g_ol_pos_deg;

        if (err_deg > PID_DEADBAND_DEG || err_deg < -PID_DEADBAND_DEG) {
            err_deg = clampf(err_deg, -MAX_DEG_PER_TICK, MAX_DEG_PER_TICK);

            int32_t steps = clampi(DegToSteps(err_deg),
                                   -(int32_t)MAX_STEPS_PER_POLL,
                                    (int32_t)MAX_STEPS_PER_POLL);
            if (steps != 0) {
                DoSteps(steps);
                g_ol_pos_deg += (float)steps * DEG_PER_STEP;
                g_last_ctrl_deg = (float)steps * DEG_PER_STEP;
            } else {
                g_last_ctrl_deg = err_deg;
            }
        } else {
            if (g_homing) {
                g_homing = 0;
                float pos = COUNTS_TO_DEG(g_enc_raw_prev);
                if (pos - STARTUP_TARGET_OFFSET_DEG > 180.0f)
                    pos -= 360.0f;
                else if (pos - STARTUP_TARGET_OFFSET_DEG < -180.0f)
                    pos += 360.0f;
                g_ol_pos_deg = pos;
            }
            Stepper_Stop();
        }
    }
}

/* --- Телеметрия и heartbeat --- */

static void TransmitAll(const uint8_t *buf, uint16_t len)
{
    if (USB_CDC_IsConnected())
        USB_CDC_Transmit(buf, len);
    UART_Transmit(buf, len);
}

static void Telemetry_Tick(uint8_t enc_ok, BiSS_Status st)
{
    static uint16_t cnt = 0;

    if (g_output_period_ms == 0)
        return;

    if (++cnt < g_output_period_ms)
        return;
    cnt = 0;

    char buf[128];
    float enc_deg = enc_ok ? COUNTS_TO_DEG(g_enc_counts) : g_ol_pos_deg;
    uint8_t ec    = enc_ok ? ERR_OK : (uint8_t)st;

    int len;
    if (g_telemetry_debug) {
        float err_deg = g_target_deg - enc_deg;
        len = snprintf(buf, sizeof(buf),
            "cp:%.2f,tp:%.2f,pe:%.2f,u:%.4f,m:%s,ec:%u,kp:%.4f,ki:%.4f,kd:%.4f\r\n",
            (double)enc_deg,
            (double)g_target_deg,
            (double)err_deg,
            (double)g_last_ctrl_deg,
            (g_mode == MODE_CLOSED_LOOP) ? "cl" : "ol",
            (unsigned)ec,
            (double)g_pid.kp,
            (double)g_pid.ki,
            (double)g_pid.kd);
    } else {
        len = snprintf(buf, sizeof(buf), "cp:%.2f,ec:%u\r\n",
            (double)enc_deg, (unsigned)ec);
    }

    if (len > 0)
        TransmitAll((uint8_t *)buf, (uint16_t)len);
}

static void Heartbeat_Tick(void)
{
    static uint32_t cnt = 0;
    if (++cnt >= LED_TOGGLE_INTERVAL) {
        cnt = 0;
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }
}

/* --- Обработка команд --- */

static void PollCommands(void)
{
    char line[64];
    Cmd_Result cmd;

    if (USB_CDC_ReadLine(line, sizeof(line)) > 0) {
        if (Cmd_Parse(line, &cmd))
            ProcessCommand(&cmd);
    }

    if (UART_ReadLine(line, sizeof(line)) > 0) {
        if (Cmd_Parse(line, &cmd))
            ProcessCommand(&cmd);
    }
}

static void SendResponse(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0)
        TransmitAll((uint8_t *)buf, (uint16_t)len);
}

static void ProcessCommand(const Cmd_Result *cmd)
{
    switch (cmd->type) {

    case CMD_ENABLE:
        g_enabled = 1;
        Stepper_SetEnable(1);
        PID_Reset(&g_pid);
        g_cl_deg_accum = 0.0f;
        g_target_deg = (g_mode == MODE_CLOSED_LOOP)
                        ? COUNTS_TO_DEG(g_enc_counts) : g_ol_pos_deg;
        g_homing = 0;
        SendResponse("ok:en\r\n");
        break;

    case CMD_DISABLE:
        g_enabled = 0;
        Stepper_Stop();
        Stepper_SetEnable(0);
        SendResponse("ok:dis\r\n");
        break;

    case CMD_SET_TARGET:
        g_target_deg = cmd->target;
        g_homing = 0;
        PID_Reset(&g_pid);
        g_cl_deg_accum = 0.0f;
        SendResponse("ok:t=%.2f\r\n", (double)g_target_deg);
        break;

    case CMD_SET_KP:
        g_pid.kp = cmd->kp;
        SendResponse("ok:kp=%.4f\r\n", (double)g_pid.kp);
        break;

    case CMD_SET_KI:
        g_pid.ki = cmd->ki;
        SendResponse("ok:ki=%.4f\r\n", (double)g_pid.ki);
        break;

    case CMD_SET_KD:
        g_pid.kd = cmd->kd;
        SendResponse("ok:kd=%.4f\r\n", (double)g_pid.kd);
        break;

    case CMD_SET_OUTPUT_PERIOD:
        g_output_period_ms = cmd->output_period_ms;
        SendResponse("ok:op=%u\r\n", (unsigned)g_output_period_ms);
        break;

    case CMD_SET_DEBUG:
        g_telemetry_debug = cmd->debug;
        SendResponse("ok:debug=%u\r\n", (unsigned)g_telemetry_debug);
        break;

    case CMD_UNKNOWN:
        SendResponse("err:unknown\r\n");
        break;

    default:
        break;
    }
}

/* --- Инициализация периферии --- */

static void BSP_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

static void PollTimer_Init(void)
{
    htim_poll.Instance               = TIM2;
    htim_poll.Init.Prescaler         = (TIM2_CLK_HZ / 10000U) - 1;
    htim_poll.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim_poll.Init.Period            = (POLL_INTERVAL_MS * 10U) - 1;
    htim_poll.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_poll.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim_poll);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
        __HAL_RCC_TIM2_CLK_ENABLE();
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL          = RCC_PLL_MUL12;
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                          RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1);

    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    pclk.UsbClockSelection    = RCC_USBCLKSOURCE_PLL;
    HAL_RCCEx_PeriphCLKConfig(&pclk);
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}
