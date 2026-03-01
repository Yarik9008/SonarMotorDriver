/* uart.h — UART для дублирования команд и телеметрии (USART1). */

#ifndef UART_H
#define UART_H

#include <stdint.h>

void     UART_Init(void);
uint8_t  UART_Transmit(const uint8_t *buf, uint16_t len);
void     UART_Task(void);
uint16_t UART_ReadLine(char *buf, uint16_t size);

#endif /* UART_H */
