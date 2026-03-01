/* stepper.h — Драйвер STEP/DIR для TMC2208 (TIM4 PWM). */

#ifndef STEPPER_H
#define STEPPER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void Stepper_Init(void);
void Stepper_SetEnable(uint8_t enabled);

/* Запустить PWM с частотой для N шагов за 1 мс. Неблокирующий. */
void Stepper_Steps(int32_t steps);

/* Остановить PWM (вызывается автоматически при следующем Stepper_Steps). */
void Stepper_Stop(void);

#endif /* STEPPER_H */
