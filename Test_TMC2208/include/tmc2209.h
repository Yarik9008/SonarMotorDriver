/* tmc2209.h — TMC2209 драйвер: UART (VACTUAL) + STEP/DIR (TIM4 PWM). */

#ifndef TMC2209_H
#define TMC2209_H

#include "board.h"

typedef enum {
    TMC_BUSY = 0,
    TMC_DONE,
    TMC_ERROR
} TMC2209_Status;

typedef enum {
    TMC_MODE_UART = 0,
    TMC_MODE_STEP_DIR
} TMC2209_Mode;

void           TMC2209_InitStart(void);
TMC2209_Status TMC2209_Poll(void);

void         TMC2209_SetMode(TMC2209_Mode mode);
TMC2209_Mode TMC2209_GetMode(void);

/* UART mode: устанавливает VACTUAL (±, микрошаги/период).
 * STEP/DIR mode: устанавливает частоту шагов (±, шагов/сек, знак = направление). */
void TMC2209_Move(int32_t value);
void TMC2209_Stop(void);
void TMC2209_SetEnable(uint8_t en);

uint8_t  TMC2209_ReadVersion(void);
uint32_t TMC2209_ReadDrvStatus(void);
uint16_t TMC2209_ReadSgResult(void);

#endif /* TMC2209_H */
