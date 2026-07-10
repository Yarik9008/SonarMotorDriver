/**
 * @file main.c
 * @brief Пример чтения LENZ IRS по BiSS-C (STM32Cube HAL, blocking).
 *
 * Плата: STM32F103C8 (Blue Pill).
 * SPI1: PA5 SCK, PA6 MISO.
 * RS-485 THVD1452: DE=PB0, RE=PB1.
 * UART1: PA9 TX — вывод в монитор порта 115200.
 */

#include "stm32f1xx_hal.h"
#include "biss_encoder/biss_encoder.h"
#include "biss_encoder/biss_models.h"
#include "biss_encoder/biss_port_stm32_hal.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef huart1;
static biss_hal_ctx_t     g_biss_hal;
static biss_encoder_t     g_enc;

static void SystemClock_Config(void);
static void UART1_Init(void);
static const char *status_str(biss_status_t st);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    UART1_Init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    memset(&g_biss_hal, 0, sizeof(g_biss_hal));
    g_biss_hal.de_port      = GPIOB;
    g_biss_hal.de_pin       = GPIO_PIN_0;
    g_biss_hal.re_port      = GPIOB;
    g_biss_hal.re_pin       = GPIO_PIN_1;
    g_biss_hal.use_dma      = 0;
    g_biss_hal.dma_irq_prio = 1;

    biss_port_t port;
    biss_port_stm32_hal_fill(&port, &g_biss_hal);

    biss_encoder_cfg_t cfg = {
        .port  = port,
        .frame = &BISS_LENZ_IRS_17BIT,
    };

    if (biss_encoder_init(&g_enc, &cfg) != BISS_OK) {
        printf("biss_encoder_init failed\r\n");
        while (1) { }
    }

    printf("BiSS-C example (STM32Cube) ready\r\n");

    while (1) {
        biss_reading_t rd;
        biss_status_t  st = biss_encoder_read(&g_enc, &rd);

        printf("st=%s pos=%lu angle=%.2f raw=0x%06lX\r\n",
               status_str(st),
               (unsigned long)rd.position,
               (double)rd.angle_deg,
               (unsigned long)rd.raw_position);

        HAL_Delay(100);
    }
}

static const char *status_str(biss_status_t st)
{
    switch (st) {
    case BISS_OK:             return "OK";
    case BISS_ERR_CRC:          return "CRC";
    case BISS_ERR_NO_RESPONSE:  return "NO_RESP";
    case BISS_ERR_SENSOR:       return "SENSOR";
    case BISS_ERR_WARNING:      return "WARN";
    case BISS_ERR_SPI:          return "SPI";
    default:                    return "?";
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    biss_port_stm32_hal_spi_msp_init(hspi, &g_biss_hal);
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    biss_port_stm32_hal_spi_msp_deinit(hspi, &g_biss_hal);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_9;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void UART1_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 100);
    return len;
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
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void Error_Handler(void)
{
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    while (1) { }
}
#endif
