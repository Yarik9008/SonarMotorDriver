/* usb_cdc.h — USB CDC (виртуальный COM-порт) для команд и телеметрии. */

#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

/* Инициализация USB стека. Вызывать при старте. */
void USB_CDC_Init(void);

/* Постановка данных в очередь TX. Возвращает 0 — OK, 1 — переполнение. */
uint8_t USB_CDC_Transmit(const uint8_t *buf, uint16_t len);

/* Задача отправки TX. Вызывать из main loop. */
void USB_CDC_Task(void);

/* Проверка подключения (1 = устройство сконфигурировано). */
uint8_t USB_CDC_IsConnected(void);

/* Перезагрузка в режим DFU (bootloader). */
void USB_CDC_RebootToDFU(void);

/* Извлечение одной строки из RX буфера. */
uint16_t USB_CDC_ReadLine(char *buf, uint16_t size);

#endif
