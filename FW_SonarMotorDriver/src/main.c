/**
 * @file main.c
 * @brief Управление шаговым двигателем с обратной связью по энкодеру BiSS-C.
 *
 * Вся математика — в градусах. Преобразование в шаги только перед DoSteps().
 * SPI-обмен с энкодером — через DMA (неблокирующий).
 *
 * Режимы:
 *   CLOSED_LOOP — PID по энкодеру (основной, удержание позиции)
 *   OPEN_LOOP   — автопереход при отвале энкодера
 */

#include "stm32f1xx_hal.h"
#include "board.h"
#include "biss_c.h"
#include "usb_cdc.h"
#include "stepper.h"
#include "pid.h"
#include "cmd_parser.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* ======================== Прототипы ======================== */

static void SystemClock_Config(void);
static void BSP_Init(void);
static void PollTimer_Init(void);
static void ProcessCommand(const Cmd_Result *cmd);
static void SendResponse(const char *fmt, ...);

/* ======================== Единицы: всё в градусах ======================== */

#define DEG_PER_STEP        (360.0f / (float)MOTOR_STEPS_PER_REV)
#define MAX_DEG_PER_TICK    ((float)MAX_SPEED_DEG_S / (float)POLL_FREQ_HZ)

static inline int32_t DegToSteps(float deg)
{
    return (int32_t)(deg / DEG_PER_STEP);
}

#define COUNTS_TO_DEG(c)    ((float)(c) * 360.0f / (float)ENCODER_COUNTS_REV)

/* ======================== Периферия ======================== */

static TIM_HandleTypeDef htim_poll;

/* ======================== Режимы ======================== */

typedef enum { MODE_CLOSED_LOOP = 0, MODE_OPEN_LOOP = 1 } CtrlMode;

/* ======================== Состояние ======================== */

static PID_State g_pid = {
    .kp = PID_KP_DEFAULT, .ki = PID_KI_DEFAULT, .kd = PID_KD_DEFAULT,
    .integral = 0.0f, .prev_error = 0.0f,
    .output_min = -MAX_DEG_PER_TICK,
    .output_max =  MAX_DEG_PER_TICK,
    .initialized = 0
};

static float    g_target_deg    = 0.0f;
static uint8_t  g_enabled       = 0;
static uint16_t g_output_period_ms = OUTPUT_PERIOD_MS_DEFAULT;

static uint32_t g_enc_raw_prev  = 0xFFFFFFFF;
static int64_t  g_enc_counts    = 0;

static float    g_cl_deg_accum  = 0.0f;

static CtrlMode g_mode          = MODE_CLOSED_LOOP;
static uint32_t g_enc_fail_cnt  = 0;
static float    g_ol_pos_deg    = 0.0f;

static uint32_t g_stats[BISS_STATUS_COUNT];
static uint32_t g_tx_busy;

/* ======================== Вспомогательные ======================== */

static void DoSteps(int32_t steps)
{
#if MOTOR_DIR_INVERT
    Stepper_Steps(-steps);
#else
    Stepper_Steps(steps);
#endif
}

/* ================================================================
 *                          MAIN
 * ================================================================ */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    BSP_Init();

    USB_CDC_Init();
    HAL_Delay(USB_ENUM_DELAY_MS);

    BiSS_Config enc_cfg = {
        .spi_instance    = SPI1,
        .resolution_bits = ENCODER_RESOLUTION_BITS,
        .de_port         = XCVR_DE_PORT,
        .de_pin          = XCVR_DE_PIN,
        .re_port         = XCVR_RE_PORT,
        .re_pin          = XCVR_RE_PIN,
    };
    BiSS_Init(&enc_cfg);
    HAL_Delay(ENCODER_STARTUP_MS);

    Stepper_Init();

    PollTimer_Init();
    HAL_TIM_Base_Start(&htim_poll);

    BiSS_Reading rd;
    char buf[160];
    uint32_t led_cnt    = 0;
    uint16_t output_cnt = 0;

    uint8_t enc_ok       = 0;
    BiSS_Status st       = BISS_ERR_NO_RESPONSE;
    uint8_t dma_pending  = 0;

    /* Первое блокирующее чтение для инициализации позиции */
    st = BiSS_Read(&rd);
    enc_ok = (st == BISS_OK || st == BISS_ERR_WARNING);
    if (enc_ok) {
        g_enc_counts   = (int64_t)rd.position;
        g_enc_raw_prev = rd.position;
        g_target_deg   = 0.0f;
        g_ol_pos_deg   = COUNTS_TO_DEG(g_enc_counts);
        g_enabled = 1;
        Stepper_SetEnable(1);
    }

    /* Запуск первого DMA-чтения */
    BiSS_StartRead();
    dma_pending = 1;

    while (1) {
        USB_CDC_Task();

        /* ---------- Парсинг команды ---------- */
        {
            char cmd_line[64];
            if (USB_CDC_ReadLine(cmd_line, sizeof(cmd_line)) > 0) {
                Cmd_Result cmd;
                if (Cmd_Parse(cmd_line, &cmd))
                    ProcessCommand(&cmd);
            }
        }

        /* ---------- Тик 1 кГц ---------- */
        if (!__HAL_TIM_GET_FLAG(&htim_poll, TIM_FLAG_UPDATE))
            continue;
        __HAL_TIM_CLEAR_FLAG(&htim_poll, TIM_FLAG_UPDATE);

        /* ---- 1. Забрать результат предыдущего DMA-чтения ---- */
        if (dma_pending && BiSS_IsReady()) {
            st = BiSS_GetResult(&rd);
            if (st < BISS_STATUS_COUNT)
                g_stats[st]++;

            enc_ok = (st == BISS_OK || st == BISS_ERR_WARNING);

            /* ---- 2. Многооборотная позиция (counts) ---- */
            if (enc_ok) {
                if (g_enc_raw_prev == 0xFFFFFFFF) {
                    g_enc_counts   = (int64_t)rd.position;
                    g_enc_raw_prev = rd.position;
                    g_target_deg   = 0.0f;
                    g_ol_pos_deg   = COUNTS_TO_DEG(g_enc_counts);
                    g_enabled = 1;
                    Stepper_SetEnable(1);
                } else {
                    int32_t delta = (int32_t)rd.position - (int32_t)g_enc_raw_prev;
                    if (delta > (int32_t)(ENCODER_COUNTS_REV / 2))
                        delta -= (int32_t)ENCODER_COUNTS_REV;
                    else if (delta < -(int32_t)(ENCODER_COUNTS_REV / 2))
                        delta += (int32_t)ENCODER_COUNTS_REV;
                    g_enc_counts   += delta;
                    g_enc_raw_prev = rd.position;
                }
            }

            dma_pending = 0;
        }

        /* ---- 3. Запуск следующего DMA-чтения (пока CPU обрабатывает PID) ---- */
        if (!dma_pending) {
            BiSS_StartRead();
            dma_pending = 1;
        }

        /* ---- 4. Автопереключение CL ↔ OL ---- */
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
                g_mode      = MODE_OPEN_LOOP;
                g_ol_pos_deg = COUNTS_TO_DEG(g_enc_counts);
            }
        }

        /* ---- 5. Управление мотором ---- */
        if (g_enabled) {
            const float dt = 1.0f / (float)POLL_FREQ_HZ;

            if (g_mode == MODE_CLOSED_LOOP && enc_ok) {
                float pos_deg = COUNTS_TO_DEG(g_enc_counts);
                float err_deg = g_target_deg - pos_deg;

                if (err_deg > PID_DEADBAND_DEG || err_deg < -PID_DEADBAND_DEG) {
                    float pid_out_deg = PID_Update(&g_pid, err_deg, dt);

                    g_cl_deg_accum += pid_out_deg;
                    if (g_cl_deg_accum > MAX_DEG_PER_TICK)
                        g_cl_deg_accum = MAX_DEG_PER_TICK;
                    else if (g_cl_deg_accum < -MAX_DEG_PER_TICK)
                        g_cl_deg_accum = -MAX_DEG_PER_TICK;

                    int32_t steps = DegToSteps(g_cl_deg_accum);
                    if (steps > (int32_t)MAX_STEPS_PER_POLL)
                        steps = (int32_t)MAX_STEPS_PER_POLL;
                    else if (steps < -(int32_t)MAX_STEPS_PER_POLL)
                        steps = -(int32_t)MAX_STEPS_PER_POLL;

                    if (steps != 0) {
                        DoSteps(steps);
                        g_cl_deg_accum -= (float)steps * DEG_PER_STEP;
                    }
                } else {
                    Stepper_Stop();
                    PID_Reset(&g_pid);
                    g_cl_deg_accum = 0.0f;
                }

            } else if (g_mode == MODE_OPEN_LOOP) {
                float err_deg = g_target_deg - g_ol_pos_deg;

                if (err_deg > PID_DEADBAND_DEG || err_deg < -PID_DEADBAND_DEG) {
                    if (err_deg > MAX_DEG_PER_TICK)
                        err_deg = MAX_DEG_PER_TICK;
                    else if (err_deg < -MAX_DEG_PER_TICK)
                        err_deg = -MAX_DEG_PER_TICK;

                    int32_t steps = DegToSteps(err_deg);
                    if (steps > (int32_t)MAX_STEPS_PER_POLL)
                        steps = (int32_t)MAX_STEPS_PER_POLL;
                    else if (steps < -(int32_t)MAX_STEPS_PER_POLL)
                        steps = -(int32_t)MAX_STEPS_PER_POLL;

                    if (steps != 0) {
                        DoSteps(steps);
                        g_ol_pos_deg += (float)steps * DEG_PER_STEP;
                    }
                } else {
                    Stepper_Stop();
                }
            }
        } else {
            Stepper_Stop();
        }

        /* ---- 6. Телеметрия ---- */
        if (g_output_period_ms > 0 && USB_CDC_IsConnected()) {
            output_cnt++;
            if (output_cnt >= g_output_period_ms) {
                output_cnt = 0;

                float enc_deg  = enc_ok ? COUNTS_TO_DEG(g_enc_counts) : g_ol_pos_deg;
                float err_deg  = g_target_deg - enc_deg;
                uint8_t err_code = enc_ok ? ERR_OK : (uint8_t)st;

                int len = snprintf(buf, sizeof(buf),
                    "cp:%.2f,tp:%.2f,pe:%.2f,m:%s,ec:%u\r\n",
                    (double)enc_deg,
                    (double)g_target_deg,
                    (double)err_deg,
                    (g_mode == MODE_CLOSED_LOOP) ? "cl" : "ol",
                    (unsigned)err_code);

                if (len > 0 && USB_CDC_Transmit((uint8_t *)buf, (uint16_t)len) != 0)
                    g_tx_busy++;
            }
        }

        /* ---- 7. Heartbeat LED ---- */
        if (++led_cnt >= LED_TOGGLE_INTERVAL) {
            led_cnt = 0;
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        }
    }
}

/* ================================================================
 *                      ОБРАБОТКА КОМАНД
 * ================================================================ */

static void SendResponse(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0 && USB_CDC_IsConnected())
        USB_CDC_Transmit((uint8_t *)buf, (uint16_t)len);
}

static void ProcessCommand(const Cmd_Result *cmd)
{
    switch (cmd->type) {

    case CMD_ENABLE:
        g_enabled = 1;
        Stepper_SetEnable(1);
        PID_Reset(&g_pid);
        g_cl_deg_accum = 0.0f;
        g_target_deg = (g_mode == MODE_CLOSED_LOOP) ? COUNTS_TO_DEG(g_enc_counts) : g_ol_pos_deg;
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

    default:
        break;
    }
}

/* ================================================================
 *                   ИНИЦИАЛИЗАЦИЯ ПЕРИФЕРИИ
 * ================================================================ */

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
