/* tmc2209.h — Неблокирующий UART-драйвер TMC2209 (ток, микрошаг). */

#ifndef TMC2209_H
#define TMC2209_H

#include "stm32f1xx_hal.h"

typedef enum {
    TMC_BUSY = 0,
    TMC_DONE,
    TMC_ERROR
} TMC2209_Status;

void           TMC2209_InitStart(void);
TMC2209_Status TMC2209_Poll(void);

void TMC2209_UartIrqHandler(void);
void TMC2209_UartTxCpltCb(UART_HandleTypeDef *h);
void TMC2209_UartRxCpltCb(UART_HandleTypeDef *h);
void TMC2209_UartErrorCb(UART_HandleTypeDef *h);

#endif /* TMC2209_H */
