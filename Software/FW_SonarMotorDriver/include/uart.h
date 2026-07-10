/**
 * @file uart.h
 * @brief Модуль командного UART (взаимодействие с ПК).
 *
 * Описывает интерфейс для неблокирующего обмена данными через USART1 (PA9/PA10)
 * и USART3 (PB10/PB11). TX дублируется на оба порта, RX объединяется.
 * Использует кольцевые буферы и DMA для минимизации нагрузки на CPU.
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/**
 * @brief Инициализация командных портов USART1 и USART3 (с DMA/IT).
 * @return 0 при успехе, -1 при ошибке (провал основного порта USART1).
 *
 * Настраивает оба порта и запускает приём в режиме ожидания. TX дублируется
 * на оба порта, RX объединяется в общий буфер.
 */
int UART_Init(void);

/* Вспомогательные функции HAL MSP для командного UART (USART1 и USART3). */
void UART_CommandMspInit(UART_HandleTypeDef *h);
void UART_CommandMspDeInit(UART_HandleTypeDef *h);

/**
 * @brief Постановка данных в очередь на отправку.
 * @param[in] buf Указатель на данные.
 * @param[in] len Количество байтов.
 * @return 0 — успешно добавлено, 1 — недостаточно места в буфере (данные отброшены).
 *
 * Данные копируются в программный кольцевой буфер и будут отправлены позже в UART_Task().
 */
uint8_t UART_Transmit(const uint8_t *buf, uint16_t len);

/* Задача отправки TX. Вызывать из main loop. */
void UART_Task(void);

/* Ненулевое значение, если любой из портов (USART1/USART3) ещё передаёт
 * или в его очереди есть данные. */
uint8_t UART_TxPending(void);

/* Извлечение одной строки из RX буфера. Возвращает длину строки или 0. */
uint16_t UART_ReadLine(char *buf, uint16_t size);

#endif /* UART_H */
