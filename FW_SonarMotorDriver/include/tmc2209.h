/* tmc2209.h — Единый app-level Motor Facade для шагового двигателя.
 *
 * Объединяет lib/tmc2209 (UART/Регистры), STEP/DIR backend (TIM4) и UART motion backend.
 * Приложение взаимодействует с мотором только через этот фасад.
 * Режим управления задаётся через MOTOR_DRIVER_MODE в board.h.
 */

#ifndef TMC2209_APP_H
#define TMC2209_APP_H

#include <stdint.h>
#include "tmc2209/tmc2209_types.h"

/* Режим управления двигателем (board.h: MOTOR_DRIVER_MODE) */
typedef enum {
    TMC2209_CONTROL_MODE_STEP_DIR = 0,  /* Движение через STEP/DIR импульсы от MCU (TIM4) */
    TMC2209_CONTROL_MODE_UART     = 1   /* Движение через VACTUAL / internal pulse generator TMC2209 */
} TMC2209_ControlMode;

/* ---- Инициализация ---- */

int TMC2209_Init(void);
uint8_t TMC2209_IsReady(void);

/* ---- Управление драйвером (единственный владелец ENN) ---- */

void TMC2209_SetEnabled(uint8_t enabled);

/* ---- Конфигурация ---- */

int TMC2209_SetCurrent(uint16_t run_ma, uint16_t hold_ma);
int TMC2209_SetMicrosteps(uint16_t microsteps);
TMC2209_ControlMode TMC2209_GetControlMode(void);

/* ---- Движение (единый API для обоих режимов) ---- */

void TMC2209_MoveSteps(int32_t steps);
void TMC2209_MoveVelocity(int32_t velocity);
void TMC2209_Stop(void);

/* ---- Диагностика ---- */

int TMC2209_GetVersion(uint8_t *version);
int TMC2209_GetDrvStatus(tmc2209_drv_status_t *st);

/* Неблокирующая cached диагностика (обновляется через TMC2209_Task) */
int TMC2209_GetCachedDrvStatus(tmc2209_drv_status_t *st);

/* Periodic task — вызывается из main loop.
 * Выполняет отложенный опрос DRV_STATUS без блокировки control loop. */
void TMC2209_Task(void);

#endif /* TMC2209_APP_H */
