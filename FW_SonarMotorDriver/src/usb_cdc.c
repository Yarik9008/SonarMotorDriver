/**
 * @file usb_cdc.c
 * @brief Реализация USB CDC (виртуальный COM-порт).
 *
 * Этот файл связывает ST USB Device Library с конкретным приложением:
 *   - Регистрирует класс CDC и его колбэки (Init, Receive, Control)
 *   - Обрабатывает команды хоста (SET/GET_LINE_CODING — скорость, формат)
 *   - Предоставляет функцию передачи данных USB_CDC_Transmit()
 *
 * Архитектура ST USB Device Library:
 *   usbd_core.c (ядро) -> usbd_cdc.c (класс CDC) -> этот файл (пользовательский интерфейс)
 *                                                     usbd_conf.c (привязка к железу)
 *                                                     usbd_desc.c (USB-дескрипторы)
 */

#include "usb_cdc.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"

/** Глобальный хэндл USB-устройства — используется всеми модулями USB */
USBD_HandleTypeDef hUsbDeviceFS;

/** Буфер приёма данных от хоста (64 байта — максимальный размер USB FS пакета) */
static uint8_t cdc_rx_buf[CDC_DATA_FS_MAX_PACKET_SIZE];

/** Текущие настройки COM-порта (запрашиваются/устанавливаются хостом).
 *  Для USB CDC реальная скорость не зависит от bitrate — передача идёт
 *  на скорости USB Full Speed (12 Мбит/с), но хост может запрашивать эти параметры. */
static USBD_CDC_LineCodingTypeDef line_coding = {
    .bitrate    = 115200,
    .format     = 0x00, /* 1 стоп-бит   */
    .paritytype = 0x00, /* Без чётности  */
    .datatype   = 0x08, /* 8 бит данных  */
};

/* ======== Колбэки интерфейса CDC ======================================= */

/** @brief Вызывается при подключении хоста — настраиваем буфер приёма. */
static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf);
    return USBD_OK;
}

/** @brief Вызывается при отключении хоста. */
static int8_t CDC_DeInit_FS(void)
{
    return USBD_OK;
}

/**
 * @brief Обработка управляющих команд от хоста (SET_LINE_CODING, GET_LINE_CODING и т.д.).
 *
 * Хост (например, терминальная программа) может запрашивать и устанавливать
 * параметры COM-порта — скорость, стоп-биты, чётность. Мы их просто сохраняем,
 * т.к. для USB CDC физическая скорость значения не имеет.
 */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)length;
    switch (cmd) {
    case CDC_SET_LINE_CODING:
        /* Хост устанавливает параметры (4 байта bitrate + format + parity + datatype) */
        line_coding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |
                                            (pbuf[2] << 16) | (pbuf[3] << 24));
        line_coding.format     = pbuf[4];
        line_coding.paritytype = pbuf[5];
        line_coding.datatype   = pbuf[6];
        break;
    case CDC_GET_LINE_CODING:
        /* Хост запрашивает текущие параметры */
        pbuf[0] = (uint8_t)(line_coding.bitrate);
        pbuf[1] = (uint8_t)(line_coding.bitrate >> 8);
        pbuf[2] = (uint8_t)(line_coding.bitrate >> 16);
        pbuf[3] = (uint8_t)(line_coding.bitrate >> 24);
        pbuf[4] = line_coding.format;
        pbuf[5] = line_coding.paritytype;
        pbuf[6] = line_coding.datatype;
        break;
    default:
        break;
    }
    return USBD_OK;
}

/**
 * @brief Вызывается при получении данных от хоста.
 *
 * Сейчас входящие данные игнорируются — энкодер работает только на передачу.
 * При необходимости здесь можно разбирать команды от ПК.
 */
static int8_t CDC_Receive_FS(uint8_t *buf, uint32_t *len)
{
    (void)buf;
    (void)len;
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS); /* Готовим следующий приём */
    return USBD_OK;
}

/** Таблица колбэков CDC-интерфейса — передаётся в библиотеку при регистрации */
static USBD_CDC_ItfTypeDef cdc_fops = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
};

/* ======== Публичный API ================================================= */

/**
 * @brief Инициализация USB CDC.
 *
 * Последовательность:
 *   1. USBD_Init() — инициализация ядра USB Device Library.
 *   2. USBD_RegisterClass() — регистрация класса CDC (дескрипторы конфигурации).
 *   3. USBD_CDC_RegisterInterface() — привязка наших колбэков.
 *   4. USBD_Start() — запуск USB-периферии, подключение к шине.
 */
void USB_CDC_Init(void)
{
    USBD_Init(&hUsbDeviceFS, &CDC_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &cdc_fops);
    USBD_Start(&hUsbDeviceFS);
}

/**
 * @brief Отправка данных в USB CDC (COM-порт).
 *
 * Неблокирующая функция: если предыдущая передача ещё не завершена
 * (TxState != 0), возвращает 1 и данные не отправляются.
 */
uint8_t USB_CDC_Transmit(const uint8_t *buf, uint16_t len)
{
    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if (hcdc == NULL)
        return 1; /* CDC ещё не инициализирован */

    if (hcdc->TxState != 0)
        return 1; /* Предыдущая передача не завершена */

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)buf, len);
    return USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK ? 0 : 1;
}

/** @brief Проверка, подключён ли USB-хост и устройство в рабочем состоянии. */
uint8_t USB_CDC_IsConnected(void)
{
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}
