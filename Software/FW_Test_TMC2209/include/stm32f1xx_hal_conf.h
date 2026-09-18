#ifndef STM32F1XX_HAL_CONF_H
#define STM32F1XX_HAL_CONF_H

/* Конфигурация STM32Cube HAL для этого проекта.
 *
 * Файл лежит в самой прошивке, а в platformio.ini включено
 * board_build.stm32cube.custom_config_header = yes. Без этого сборка брала бы
 * общий stm32f1xx_hal_conf.h из каталога пакета framework-stm32cubef1, а
 * прошивки со своим заголовком этот общий файл при каждой сборке удаляют —
 * тогда сборка стенда падала бы с «Implicit dependency stm32f1xx_hal_conf.h
 * not found» в зависимости от того, какой проект собрали перед ним. Свой
 * заголовок делает сборку герметичной: она не зависит ни от соседних
 * проектов, ни от порядка сборки.
 *
 * Значения повторяют шаблон ST (stm32f1xx_hal_conf_template.h), из которого
 * PlatformIO раньше генерировал общий файл, поэтому прошивка собирается
 * байт-в-байт такой же. Отличие одно: включены только модули, которые нужны
 * стенду, — остальные всё равно отбрасывались компоновщиком (--gc-sections).
 */

/* -------- Выбор модулей -------- */
/* Стенд использует GPIO (STEP/DIR/ENN), USART1 (CLI) и USART2 (PDN_UART),
 * TIM4 (STEP) и системные модули. DMA включён потому, что на его типы
 * ссылаются дескрипторы UART и TIM. */
#define HAL_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* -------- Параметры генераторов -------- */
#if !defined(HSE_VALUE)
#define HSE_VALUE            8000000U
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT  100U
#endif

#if !defined(HSI_VALUE)
#define HSI_VALUE            8000000U
#endif

#if !defined(LSE_VALUE)
#define LSE_VALUE            32768U
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT  5000U
#endif

#if !defined(LSI_VALUE)
#define LSI_VALUE            40000U
#endif

/* -------- Системные параметры -------- */
#define VDD_VALUE                    3300U
#define TICK_INT_PRIORITY            15U
#define USE_RTOS                     0U
#define PREFETCH_ENABLE              1U

/* -------- Подключение модулей HAL -------- */
#ifdef HAL_RCC_MODULE_ENABLED
 #include "stm32f1xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
 #include "stm32f1xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
 #include "stm32f1xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
 #include "stm32f1xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
 #include "stm32f1xx_hal_flash.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32f1xx_hal_tim.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f1xx_hal_uart.h"
#endif

/* -------- Проверки (Assert) -------- */
/* #define USE_FULL_ASSERT 1U */
#ifdef USE_FULL_ASSERT
 #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
 void assert_failed(uint8_t *file, uint32_t line);
#else
 #define assert_param(expr) ((void)0U)
#endif

#endif /* STM32F1XX_HAL_CONF_H */
