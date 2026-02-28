// Точка входа прошивки — периодический опрос энкодера LENZ IRS по таймеру.
//
// Архитектура (неблокирующая):
//   - TIM2 отсчитывает POLL_INTERVAL_MS (2 мс → 500 Гц).
//   - Главный цикл опрашивает флаг переполнения таймера (Polling Timer).
//   - При установленном флаге: чтение энкодера по SPI, отправка данных в кольцевой буфер USB.
//   - Фоновая функция `USB_CDC_Task()` выкачивает буфер и шлёт данные на ПК.
//   - LED мигает с частотой ~1 Гц (heartbeat), независимо от частоты опроса.
//
// Тактирование: HSI/2 (4 МГц) → PLL ×12 → SYSCLK 48 МГц.
// USB получает 48 МГц от PLL напрямую (делитель 1).

#include "stm32f1xx_hal.h"
#include "board.h"
#include "biss_c.h"
#include "usb_cdc.h"
#include <stdio.h>
#include <string.h>

// Прототипы локальных функций

static void SystemClock_Config(void);
static void BSP_Init(void);
static void PollTimer_Init(void);
static void FormatReading(char *buf, size_t size, int *out_len,
                          const BiSS_Reading *rd, BiSS_Status st);

// Состояние модуля

static TIM_HandleTypeDef htim_poll;

// Счётчики событий — диагностика качества связи (доступны через отладчик)
static uint32_t g_stats[BISS_STATUS_COUNT];
static uint32_t g_tx_busy;  // Количество пропущенных TX (USB CDC занят)

// Точка входа

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

    PollTimer_Init();

    BiSS_Reading rd;
    char buf[128];
    uint32_t led_cnt = 0;

    /* Запускаем таймер без прерываний */
    HAL_TIM_Base_Start(&htim_poll);

    while (1) {
        /* Фоновая обработка отправки данных по USB CDC (из кольцевого буфера) */
        USB_CDC_Task();

        /* Проверяем аппаратный флаг переполнения таймера (UIF) */
        if (__HAL_TIM_GET_FLAG(&htim_poll, TIM_FLAG_UPDATE)) {
            __HAL_TIM_CLEAR_FLAG(&htim_poll, TIM_FLAG_UPDATE);

            BiSS_Status st = BiSS_Read(&rd);

            if (st < BISS_STATUS_COUNT)
                g_stats[st]++;

            int len;
            FormatReading(buf, sizeof(buf), &len, &rd, st);

            if (USB_CDC_IsConnected() && len > 0) {
                if (USB_CDC_Transmit((uint8_t *)buf, (uint16_t)len) != 0)
                    g_tx_busy++;
            }

            if (++led_cnt >= LED_TOGGLE_INTERVAL) {
                led_cnt = 0;
                HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
            }
        }
    }
}

// Форматирование результата

// Формирует текстовую строку с результатом чтения энкодера.
// При успехе: позиция, угол в градусах, сырое 24-битное значение.
// При ошибке: мнемоника статуса, флаги E/W, сырой SPI-дамп.
static void FormatReading(char *buf, size_t size, int *out_len,
                          const BiSS_Reading *rd, BiSS_Status st)
{
    if (st == BISS_OK || st == BISS_ERR_WARNING) {
        *out_len = snprintf(buf, size,
                "POS: %lu  ANGLE: %.3f deg  (raw24: 0x%06lX)%s\r\n",
                (unsigned long)rd->position,
                (double)rd->angle_deg,
                (unsigned long)rd->raw_position,
                st == BISS_ERR_WARNING ? "  [W]" : "");
    } else {
        *out_len = snprintf(buf, size,
                "ERR:%s  E=%u W=%u  raw=0x%06lX  SPI[%02X %02X %02X %02X %02X %02X]\r\n",
                BiSS_StatusStr(st), rd->error, rd->warning,
                (unsigned long)rd->raw_position,
                rd->spi_dump[0], rd->spi_dump[1], rd->spi_dump[2],
                rd->spi_dump[3], rd->spi_dump[4], rd->spi_dump[5]);
    }
}

// Инициализация платы (BSP)

// Включение тактирования GPIO и настройка светодиода.
// Тактирование GPIO-портов включается здесь централизованно,
// до инициализации периферии (USB, SPI, трансивер).
static void BSP_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();   /* PA5/PA6 — SPI1, PA11/PA12 — USB */
    __HAL_RCC_GPIOB_CLK_ENABLE();   /* PB0/PB1 — DE/RE трансивера */
    __HAL_RCC_GPIOC_CLK_ENABLE();   /* PC13 — LED */

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); /* LED off (active low) */
}

// Таймер опроса (TIM2)

// Инициализация TIM2 для периодического опроса энкодера.
// TIM2 тактируется от APB1 ×2 = 48 МГц (APB1 prescaler > 1).
// Prescaler переводит таймер в тики по 1 мс (48000000 / 48000 = 1 кГц).
// Period = POLL_INTERVAL_MS − 1, что даёт ровно POLL_FREQ_HZ Гц.
static void PollTimer_Init(void)
{
    htim_poll.Instance               = TIM2;
    htim_poll.Init.Prescaler         = (TIM2_CLK_HZ / 1000U) - 1;  /* → 1 кГц тик */
    htim_poll.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim_poll.Init.Period            = POLL_INTERVAL_MS - 1;
    htim_poll.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_poll.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&htim_poll);
}

// Инициализация аппаратных ресурсов таймера (вызывается из HAL_TIM_Base_Init)
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        /* Прерывания отключаем — будем опрашивать флаг Update (UIF) вручную */
    }
}

// Тактирование

// Настройка тактирования от внутреннего RC-генератора (HSI).
// HSI (8 МГц) / 2 → PLL ×12 → SYSCLK = 48 МГц.
// 
// На HSI невозможно получить 72 МГц одновременно с 48 МГц для USB:
// PLL принимает HSI/2 = 4 МГц, множитель макс. ×16 → макс. 64 МГц.
// 4 × 12 = 48 МГц — единственный вариант, дающий ровно 48 МГц для USB FS.
//
// Шины: AHB = 48, APB1 = 24, APB2 = 48, USB = 48 МГц.
// Flash: 1 wait state (LATENCY_1) для 48 МГц.
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

// SysTick

// Обработчик SysTick (1 мс). Необходим для HAL_Delay() и HAL_GetTick().
void SysTick_Handler(void)
{
    HAL_IncTick();
}
