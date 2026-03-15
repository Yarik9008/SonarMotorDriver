/* uart.h — UART (USART1) для дублирования команд и телеметрии на ПК.
 *
 * PA9: TX, PA10: RX. RX — DMA circular + IDLE line; TX — DMA через ring buffer.
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>

/* Инициализация USART1 и DMA. Вызывать один раз при старте. 
 * Возвращает 0 при успехе, -1 при ошибке. */
int UART_Init(void);

/* Постановка данных в очередь TX. Возвращает 0 — OK, 1 — буфер переполнен. */
uint8_t UART_Transmit(const uint8_t *buf, uint16_t len);

/* Задача отправки TX. Вызывать из main loop. */
void UART_Task(void);

/* Извлечение одной строки из RX буфера. Возвращает длину строки или 0. */
uint16_t UART_ReadLine(char *buf, uint16_t size);

#endif /* UART_H */
