#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include "usbd_def.h"

#ifndef DEVICE_FS
#define DEVICE_FS 0
#endif

extern USBD_HandleTypeDef hUsbDeviceFS;

/** Инициализация USB-стека и старт виртуального COM-порта. */
void MX_USB_DEVICE_Init(void);

/**
 * Ждать перечисления хостом до timeout_ms.
 * @return 1, если порт готов, иначе 0.
 */
int usb_device_wait_ready(uint32_t timeout_ms);

#endif /* USB_DEVICE_H */
