#ifndef STM32F1XX_HAL_CONF_H
#define STM32F1XX_HAL_CONF_H

#define HAL_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

#define HSE_VALUE            8000000U
#define HSE_STARTUP_TIMEOUT  100U
#define HSI_VALUE            8000000U
#define LSE_VALUE            32768U
#define LSE_STARTUP_TIMEOUT  5000U
#define LSI_VALUE            40000U

#define VDD_VALUE                    3300U
#define TICK_INT_PRIORITY            15U
#define USE_RTOS                     0U
#define PREFETCH_ENABLE              1U

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif

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
#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32f1xx_hal_pwr.h"
#endif
#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32f1xx_hal_spi.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f1xx_hal_uart.h"
#endif

#include "stm32f1xx_hal_def.h"

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#endif

#endif /* STM32F1XX_HAL_CONF_H */
