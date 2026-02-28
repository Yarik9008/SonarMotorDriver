/**
 * @file stepper.h
 * @brief Драйвер шагового двигателя через TMC2208 (интерфейс STEP/DIR).
 *
 * TMC2208 принимает импульсы STEP и направление DIR.
 * Минимальная длительность импульса STEP: ~1–2 мкс (по даташиту TMC2208).
 *
 * Stepper_Steps() — неблокирующий: запускает PWM и возвращает управление.
 * PWM работает до следующего вызова Stepper_Steps() или Stepper_Stop().
 */

#ifndef STEPPER_H
#define STEPPER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void Stepper_Init(void);
void Stepper_SetEnable(uint8_t enabled);
void Stepper_SetDir(uint8_t dir_cw);
void Stepper_Step(void);

/** @brief Запустить PWM с частотой для N шагов за 1 мс. Неблокирующий. */
void Stepper_Steps(int32_t steps);

/** @brief Остановить PWM (вызывается автоматически при следующем Stepper_Steps). */
void Stepper_Stop(void);

#endif /* STEPPER_H */
