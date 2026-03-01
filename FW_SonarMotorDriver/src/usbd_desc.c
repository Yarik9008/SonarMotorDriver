/* usbd_desc.c — USB-дескрипторы устройства (VID/PID, строки). */

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

#define USBD_VID                  0x0483
#define USBD_PID_FS               0x5740
#define USBD_LANGID_STRING        0x0409
#define USBD_MANUFACTURER_STRING  "SonarMotorDriver"
#define USBD_PRODUCT_STRING       "BiSS Encoder CDC"
#define USBD_SERIAL_STRING        "00001"
#define USBD_CONFIGURATION_STRING "CDC Config"
#define USBD_INTERFACE_STRING     "CDC Interface"

static uint8_t *USBD_CDC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_CDC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef CDC_Desc = {
    USBD_CDC_DeviceDescriptor,
    USBD_CDC_LangIDStrDescriptor,
    USBD_CDC_ManufacturerStrDescriptor,
    USBD_CDC_ProductStrDescriptor,
    USBD_CDC_SerialStrDescriptor,
    USBD_CDC_ConfigStrDescriptor,
    USBD_CDC_InterfaceStrDescriptor,
};

static uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __attribute__((aligned(4))) = {
    USB_LEN_DEV_DESC,
    USB_DESC_TYPE_DEVICE,
    0x00, 0x02,                                    /* USB 2.00 */
    0x02,                                          /* CDC */
    0x02,                                          /* ACM */
    0x00,
    USB_MAX_EP0_SIZE,
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),
    LOBYTE(USBD_PID_FS), HIBYTE(USBD_PID_FS),
    0x00, 0x02,
    USBD_IDX_MFC_STR,
    USBD_IDX_PRODUCT_STR,
    USBD_IDX_SERIAL_STR,
    USBD_MAX_NUM_CONFIGURATION,
};

static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __attribute__((aligned(4))) = {
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING),
};

static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __attribute__((aligned(4)));

/* --- Геттеры дескрипторов --- */

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

static uint8_t *USBD_CDC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

static uint8_t *USBD_CDC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

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
