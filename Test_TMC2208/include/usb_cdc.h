/* usb_cdc.h — USB CDC (виртуальный COM-порт). */

#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

void     USB_CDC_Init(void);
uint8_t  USB_CDC_Transmit(const uint8_t *buf, uint16_t len);
void     USB_CDC_Task(void);
uint8_t  USB_CDC_IsConnected(void);
void     USB_CDC_RebootToDFU(void);
uint16_t USB_CDC_ReadLine(char *buf, uint16_t size);

#endif
