/**
 * @file main.c
 * @brief Главный файл прошивки — опрос энкодера LENZ IRS и вывод данных по USB.
 *
 * Общий алгоритм работы:
 *   1. Настройка тактирования: HSI/2 (4 МГц) -> PLL x12 -> SYSCLK 48 МГц,
 *      USB получает 48 МГц напрямую от PLL (делитель 1).
 *   2. Инициализация USB CDC — устройство определяется на ПК как COM-порт.
 *   3. Инициализация драйвера BiSS C (SPI1) для связи с энкодером.
 *   4. В бесконечном цикле: каждые 100 мс считываем позицию с энкодера
 *      и отправляем строку с данными (или кодом ошибки) в COM-порт.
 */

#include "stm32f1xx_hal.h"
#include "biss_c.h"
#include "usb_cdc.h"
#include <stdio.h>
#include <string.h>

static void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Включаем тактирование портов GPIO, которые будут использоваться */
    __HAL_RCC_GPIOA_CLK_ENABLE();  /* PA5/PA6 — SPI1, PA11/PA12 — USB */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();  /* PC13 — светодиод на плате Blue Pill */

    /* Настройка встроенного светодиода PC13 (активный уровень — LOW) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_13;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Инициализация USB CDC — после этого устройство появится как COM-порт.
     * Пауза 1500 мс даёт хосту время на энумерацию (определение устройства). */
    USB_CDC_Init();
    HAL_Delay(1500);

    /*
     * Настройка драйвера энкодера.
     * resolution_bits = 17 для моделей IRS-I34/I50/I60.
     * Для моделей IRS-I70/I80/I90 поставить 18.
     *
     * Управление трансивером THVD1452:
     *   PB0 = DE (Driver Enable)  — HIGH включает передатчик (MA clock)
     *   PB1 = RE (Receiver Enable) — LOW включает приёмник (SLO data)
     */
    BiSS_Config enc_cfg = {
        .spi_instance    = SPI1,
        .resolution_bits = 17,
        .de_port         = GPIOB,
        .de_pin          = GPIO_PIN_0,
        .re_port         = GPIOB,
        .re_pin          = GPIO_PIN_1,
    };
    BiSS_Init(&enc_cfg);
    HAL_Delay(50); /* Время готовности энкодера после подачи питания — 50 мс */

    BiSS_Reading rd;
    char buf[128];

    while (1) {
        /* Считываем текущую позицию с энкодера по протоколу BiSS C */
        BiSS_Status st = BiSS_Read(&rd);

        /* Формируем текстовую строку для отправки по USB */
        int len;
        if (st == BISS_OK) {
            /* Успешное чтение — выводим позицию, угол и сырые данные */
            len = snprintf(buf, sizeof(buf),
                           "POS: %lu  ANGLE: %.3f deg  (raw24: 0x%06lX)\r\n",
                           (unsigned long)rd.position,
                           (double)rd.angle_deg,
                           (unsigned long)rd.raw_position);
        } else {
            /* Ошибка — выводим код ошибки, биты Error/Warning и сырые данные */
            len = snprintf(buf, sizeof(buf),
                           "ERR:%d  E=%u W=%u  raw=0x%06lX\r\n",
                           (int)st, rd.error, rd.warning,
                           (unsigned long)rd.raw_position);
        }

        /* Отправляем строку в USB CDC, только если хост подключён */
        if (USB_CDC_IsConnected() && len > 0)
            USB_CDC_Transmit((uint8_t *)buf, (uint16_t)len);

        /* Мигаем светодиодом — визуальная индикация работы основного цикла */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100); /* Период опроса ~10 Гц */
    }
}

/**
 * @brief Настройка системного тактирования от внутреннего RC-генератора (HSI).
 *
 * Цепочка: HSI (8 МГц) / 2 = 4 МГц -> PLL (×12) -> SYSCLK = 48 МГц.
 *
 * Внешний кварц (HSE) не требуется. HSI менее точен (±1%), но для USB
 * допустимо — STM32F103 имеет внутреннюю калибровку HSI.
 *
 * Ограничение: на HSI невозможно получить 72 МГц с USB 48 МГц,
 * т.к. PLL от HSI принимает HSI/2 = 4 МГц, а максимальный множитель ×16.
 * 4 × 12 = 48 МГц — единственная частота, дающая ровно 48 МГц для USB.
 *
 * Шины:
 *   AHB  = 48 МГц (без делителя)
 *   APB1 = 24 МГц (делитель /2, макс. для APB1 = 36 МГц)
 *   APB2 = 48 МГц (без делителя, от неё тактируется SPI1)
 *   USB  = 48 МГц (PLL / 1 — без деления)
 *
 * Flash: 1 цикл ожидания (LATENCY_1) для работы на 48 МГц.
 */
static void SystemClock_Config(void)
{
    /* Настройка источника тактирования — внутренний RC-генератор + PLL */
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2; /* HSI/2 = 4 МГц на входе PLL */
    osc.PLL.PLLMUL          = RCC_PLL_MUL12;          /* 4 МГц × 12 = 48 МГц */
    HAL_RCC_OscConfig(&osc);

    /* Настройка делителей шин */
    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                          RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* AHB  = 48 МГц */
    clk.APB1CLKDivider = RCC_HCLK_DIV2;     /* APB1 = 24 МГц */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2 = 48 МГц */
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1);

    /* Тактирование USB: 48 МГц / 1 = 48 МГц (стандарт USB Full Speed) */
    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    pclk.UsbClockSelection    = RCC_USBCLKSOURCE_PLL;
    HAL_RCCEx_PeriphCLKConfig(&pclk);
}

/**
 * @brief Обработчик прерывания SysTick (вызывается каждую 1 мс).
 *
 * Необходим для работы HAL_Delay() и внутреннего счётчика тиков HAL.
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
