/**
 * @file  usbd_desc.c
 * @brief Дескрипторы USB-устройства (виртуальный COM-порт, класс CDC).
 */
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

#define USBD_VID                0x0483U        /* STMicroelectronics */
#define USBD_PID                0x5740U        /* STM32 Virtual COM Port */
#define USBD_LANGID_STRING      0x0409U        /* English (US) */
#define USBD_MANUFACTURER       "DIY"
#define USBD_PRODUCT_STRING     "AS5047P encoder test"
#define USBD_CONFIGURATION_STR  "CDC Config"
#define USBD_INTERFACE_STR      "CDC Interface"

#define USB_SIZ_STRING_SERIAL   0x1AU          /* 12 байт UID -> 24 симв. UTF-16 + заголовок */
#define UID_BASE_F1             0x1FFFF7E8U    /* 96-битный уникальный ID STM32F1 */

static uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef FS_Desc = {
    USBD_FS_DeviceDescriptor,
    USBD_FS_LangIDStrDescriptor,
    USBD_FS_ManufacturerStrDescriptor,
    USBD_FS_ProductStrDescriptor,
    USBD_FS_SerialStrDescriptor,
    USBD_FS_ConfigStrDescriptor,
    USBD_FS_InterfaceStrDescriptor,
};

/* Дескриптор устройства: класс задаётся на уровне интерфейса (CDC) */
__ALIGN_BEGIN static uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
    0x12,                       /* bLength */
    USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
    0x00, 0x02,                 /* bcdUSB = 2.00 */
    0x02,                       /* bDeviceClass: CDC */
    0x00,                       /* bDeviceSubClass */
    0x00,                       /* bDeviceProtocol */
    USB_MAX_EP0_SIZE,           /* bMaxPacketSize0 */
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),
    LOBYTE(USBD_PID), HIBYTE(USBD_PID),
    0x00, 0x02,                 /* bcdDevice = 2.00 */
    USBD_IDX_MFC_STR,           /* iManufacturer */
    USBD_IDX_PRODUCT_STR,       /* iProduct */
    USBD_IDX_SERIAL_STR,        /* iSerialNumber */
    USBD_MAX_NUM_CONFIGURATION  /* bNumConfigurations */
};

__ALIGN_BEGIN static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING)
};

__ALIGN_BEGIN static uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
    USB_SIZ_STRING_SERIAL,
    USB_DESC_TYPE_STRING,
};

__ALIGN_BEGIN static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

/** Полубайт -> символ UTF-16LE (для серийного номера из UID). */
static void uid_to_unicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        uint8_t nibble = (uint8_t)((value >> (28U - (4U * i))) & 0x0FU);

        pbuf[2U * i]      = (nibble < 0x0AU) ? (uint8_t)(nibble + 0x30U)   /* '0'..'9' */
                                             : (uint8_t)(nibble + 0x37U);  /* 'A'..'F' */
        pbuf[2U * i + 1U] = 0;
    }
}

static void serial_from_uid(void)
{
    uint32_t id0 = *(volatile uint32_t *)(UID_BASE_F1);
    uint32_t id1 = *(volatile uint32_t *)(UID_BASE_F1 + 4U);
    uint32_t id2 = *(volatile uint32_t *)(UID_BASE_F1 + 8U);

    id0 += id2;
    uid_to_unicode(id0, &USBD_StringSerial[2], 8);
    uid_to_unicode(id1, &USBD_StringSerial[18], 4);
}

static uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_FS_DeviceDesc);
    return USBD_FS_DeviceDesc;
}

static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_LangIDDesc);
    return USBD_LangIDDesc;
}

static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_MANUFACTURER, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = USB_SIZ_STRING_SERIAL;
    serial_from_uid();
    return USBD_StringSerial;
}

static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STR, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_INTERFACE_STR, USBD_StrDesc, length);
    return USBD_StrDesc;
}
