/**
 * @file usbd_desc.c
 * @brief USB-дескрипторы устройства (идентификация на хосте).
 *
 * Дескрипторы — это структуры данных, которые устройство отправляет хосту
 * при энумерации. По ним хост понимает, что это за устройство:
 *
 *   Device Descriptor — VID/PID, класс устройства (CDC), версия USB.
 *   String Descriptors — текстовые строки (производитель, название, серийный номер).
 *
 * VID 0x0483, PID 0x5740 — стандартная пара ST для USB CDC (виртуальный COM).
 * Драйвер на Windows определяется автоматически (CDC ACM).
 */

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

#define USBD_VID                  0x0483  /**< Vendor ID (STMicroelectronics) */
#define USBD_PID_FS               0x5740  /**< Product ID (Virtual COM Port) */
#define USBD_LANGID_STRING        0x0409  /**< Язык строковых дескрипторов: en-US */
#define USBD_MANUFACTURER_STRING  "SonarMotorDriver"
#define USBD_PRODUCT_STRING       "BiSS Encoder CDC"
#define USBD_SERIAL_STRING        "00001"
#define USBD_CONFIGURATION_STRING "CDC Config"
#define USBD_INTERFACE_STRING     "CDC Interface"

/* Прототипы функций-геттеров дескрипторов */
static uint8_t *USBD_CDC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

/**
 * Таблица указателей на функции-геттеры дескрипторов.
 * Ядро USB вызывает нужную функцию в ответ на GET_DESCRIPTOR от хоста.
 */
USBD_DescriptorsTypeDef CDC_Desc = {
    USBD_CDC_DeviceDescriptor,
    USBD_CDC_LangIDStrDescriptor,
    USBD_CDC_ManufacturerStrDescriptor,
    USBD_CDC_ProductStrDescriptor,
    USBD_CDC_SerialStrDescriptor,
    USBD_CDC_ConfigStrDescriptor,
    USBD_CDC_InterfaceStrDescriptor,
};

/**
 * Дескриптор устройства (18 байт) — первое, что запрашивает хост.
 * Описывает: версию USB, класс (0x02 = CDC), VID/PID, количество конфигураций.
 */
static uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __attribute__((aligned(4))) = {
    USB_LEN_DEV_DESC,           /* bLength — размер дескриптора (18 байт)  */
    USB_DESC_TYPE_DEVICE,       /* bDescriptorType — тип: Device           */
    0x00, 0x02,                 /* bcdUSB — версия USB 2.00                */
    0x02,                       /* bDeviceClass — CDC (Communications)     */
    0x02,                       /* bDeviceSubClass — Abstract Control Model */
    0x00,                       /* bDeviceProtocol                         */
    USB_MAX_EP0_SIZE,           /* bMaxPacketSize0 — макс. пакет EP0 (64)  */
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),     /* idVendor  = 0x0483 */
    LOBYTE(USBD_PID_FS), HIBYTE(USBD_PID_FS), /* idProduct = 0x5740 */
    0x00, 0x02,                 /* bcdDevice — версия устройства 2.00      */
    USBD_IDX_MFC_STR,          /* iManufacturer — индекс строки            */
    USBD_IDX_PRODUCT_STR,      /* iProduct — индекс строки                 */
    USBD_IDX_SERIAL_STR,       /* iSerialNumber — индекс строки            */
    USBD_MAX_NUM_CONFIGURATION, /* bNumConfigurations = 1                  */
};

/** Дескриптор языка — указывает, что строки на английском (en-US). */
static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __attribute__((aligned(4))) = {
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING),
};

/** Общий буфер для формирования строковых дескрипторов (UTF-16LE). */
static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __attribute__((aligned(4)));

/* ======== Функции-геттеры дескрипторов ================================== */

static uint8_t *USBD_CDC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_DeviceDesc);
    return USBD_DeviceDesc;
}

static uint8_t *USBD_CDC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_LangIDDesc);
    return USBD_LangIDDesc;
}

/** @brief Возвращает строку производителя: "SonarMotorDriver". */
static uint8_t *USBD_CDC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

/** @brief Возвращает название продукта: "BiSS Encoder CDC". */
static uint8_t *USBD_CDC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

/** @brief Возвращает серийный номер. */
static uint8_t *USBD_CDC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_SERIAL_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *USBD_CDC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *USBD_CDC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}
