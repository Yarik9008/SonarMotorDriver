/**
 * @file  stm32f1xx_it.c
 * @brief Обработчики прерываний. Остальные остаются слабыми (Default_Handler)
 *        из startup-файла фреймворка.
 */
#include "stm32f1xx_hal.h"

extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim2;

/** Системный таймер HAL: HAL_GetTick() / HAL_Delay(). */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/** Такт опроса энкодера: чтение по SPI + шаг фильтра (см. main.c). */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/** Низкоприоритетное прерывание USB (у F103 делит вектор с CAN1 RX0). */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

void HardFault_Handler(void)
{
    for (;;) {
        /* останов: смотреть отладчиком */
    }
}
