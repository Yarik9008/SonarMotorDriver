/**
 * @file usb_cdc.h
 * @brief Высокоуровневый API для USB CDC (виртуальный COM-порт).
 *
 * Обёртка над ST USB Device Library — скрывает детали инициализации
 * и предоставляет три простые функции: Init, Transmit, IsConnected.
 */

#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

/** @brief Инициализация USB Device как CDC (COM-порт). */
void    USB_CDC_Init(void);

/**
 * @brief Отправка данных в COM-порт.
 * @param buf Указатель на буфер с данными.
 * @param len Длина данных в байтах.
 * @return 0 при успехе, 1 если устройство занято или не готово.
 */
uint8_t USB_CDC_Transmit(const uint8_t *buf, uint16_t len);

/**
 * @brief Проверка, подключён ли хост (ПК открыл COM-порт).
 * @return 1 если устройство в состоянии CONFIGURED, 0 иначе.
 */
uint8_t USB_CDC_IsConnected(void);

#endif
