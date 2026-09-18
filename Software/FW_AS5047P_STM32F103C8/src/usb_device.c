/**
 * @file  usb_device.c
 * @brief Инициализация USB-устройства (CDC, виртуальный COM-порт).
 */
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "stm32f1xx_hal.h"

USBD_HandleTypeDef hUsbDeviceFS;

/**
 * Принудительное переподключение к хосту.
 * После сброса МК (без снятия питания с USB) хост считает устройство прежним
 * и не перечисляет его заново. Кратко прижимаем D+ (PA12) к земле — для хоста
 * это выглядит как отключение кабеля.
 */
static void usb_force_reenumeration(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin   = GPIO_PIN_12;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(50);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);   /* дальше линией управляет периферия USB */
}

void MX_USB_DEVICE_Init(void)
{
    usb_force_reenumeration();

    if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK) {
        return;
    }
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) {
        return;
    }
    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK) {
        return;
    }
    USBD_Start(&hUsbDeviceFS);
}

int usb_device_wait_ready(uint32_t timeout_ms)
{
    uint32_t started = HAL_GetTick();

    while ((HAL_GetTick() - started) < timeout_ms) {
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            return 1;
        }
    }
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1 : 0;
}
