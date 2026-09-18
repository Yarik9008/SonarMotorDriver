#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

#include "usbd_cdc.h"

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/** Отправка буфера в виртуальный COM-порт (не блокирует надолго). */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/**
 * Запись лога в USB CDC.
 * Возвращает len при успехе или -1, если хост не готов (порт не открыт) —
 * в этом случае данные молча теряются, прошивка не зависает.
 */
int usb_cdc_write(const uint8_t *data, uint16_t len);

/** true, если устройство сконфигурировано хостом (перечисление прошло). */
int usb_cdc_ready(void);

#endif /* USBD_CDC_IF_H */
