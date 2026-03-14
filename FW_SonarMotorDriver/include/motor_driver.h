/* motor_driver.h — Единый high-level API управления шаговым двигателем.
 *
 * Объединяет TMC2209 (UART), STEP/DIR backend и UART motion backend.
 * Приложение не знает о stepper.c, tmc2209.c — только MotorDriver_*().
 * Режим управления задаётся в board.h (MOTOR_DRIVER_MODE).
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include "tmc2209/tmc2209_types.h"

/* Режим управления двигателем (board.h: MOTOR_DRIVER_MODE) */
typedef enum {
    MOTOR_DRIVER_MODE_STEP_DIR = 0,  /* STEP/DIR импульсы от MCU (TIM4) */
    MOTOR_DRIVER_MODE_UART      = 1   /* VACTUAL / internal pulse generator TMC2209 */
} motor_driver_mode_t;

/* ---- Инициализация ---- */

void MotorDriver_Init(void);
uint8_t MotorDriver_IsReady(void);

/* ---- Управление драйвером (единственный владелец ENN) ---- */

void MotorDriver_SetEnabled(uint8_t enabled);

/* ---- Конфигурация ---- */

int MotorDriver_ConfigureCurrent(uint16_t run_ma, uint16_t hold_ma);
int MotorDriver_ConfigureMicrosteps(uint16_t microsteps);
motor_driver_mode_t MotorDriver_GetControlMode(void);

/* ---- Движение (единый API для обоих режимов) ---- */

void MotorDriver_MoveSteps(int32_t steps);
void MotorDriver_MoveVelocity(int32_t velocity);
void MotorDriver_Stop(void);

/* ---- Диагностика ---- */

int MotorDriver_GetVersion(uint8_t *version);
int MotorDriver_GetDrvStatus(tmc2209_drv_status_t *st);

#endif /* MOTOR_DRIVER_H */
