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
 * @brief Отправка данных в COM-порт (помещение в кольцевой буфер).
 * @param buf Указатель на буфер с данными.
 * @param len Длина данных в байтах.
 * @return 0 при успехе, 1 если буфер переполнен или устройство не готово.
 */
uint8_t USB_CDC_Transmit(const uint8_t *buf, uint16_t len);

/**
 * @brief Фоновая задача обработки кольцевого буфера.
 *
 * Должна вызываться в главном цикле. Извлекает данные из буфера
 * и отправляет их в USB, если интерфейс свободен.
 */
void    USB_CDC_Task(void);

/**
 * @brief Проверка, подключён ли хост (ПК открыл COM-порт).
 * @return 1 если устройство в состоянии CONFIGURED, 0 иначе.
 */
uint8_t USB_CDC_IsConnected(void);

/**
 * @brief Перезагрузка в DFU-загрузчик.
 *
 * Записывает маркер 0x424C в BKP DR10 и выполняет системный сброс.
 * Совместимо с STM32duino bootloader (generic_boot20).
 */
void USB_CDC_RebootToDFU(void);

/**
 * @brief Чтение строки из буфера приёма (до \r или \n).
 * @param buf  Буфер для записи (с завершающим \0).
 * @param size Размер буфера.
 * @return Длина прочитанных символов (без \r\n), 0 если строка не готова.
 */
uint16_t USB_CDC_ReadLine(char *buf, uint16_t size);

#endif
