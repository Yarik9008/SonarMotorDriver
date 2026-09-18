/**
 * @file  usbd_cdc_if.c
 * @brief Интерфейс класса CDC: приём/передача данных виртуального COM-порта.
 */
#include "usbd_cdc_if.h"
#include "usb_device.h"

#define APP_RX_DATA_SIZE  CDC_DATA_FS_MAX_PACKET_SIZE
#define APP_TX_DATA_SIZE  CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_TX_TIMEOUT_MS 10U   /* хост не забирает данные -> строку теряем */

static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* Параметры "порта" — реальному железу не соответствуют, но хост их запрашивает */
static uint8_t line_coding[7] = {
    0x00, 0xC2, 0x01, 0x00,  /* 115200 бод */
    0x00,                    /* 1 стоп-бит */
    0x00,                    /* без контроля чётности */
    0x08                     /* 8 бит данных */
};

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS
};

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    return USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    switch (cmd) {
    case CDC_SET_LINE_CODING:
        if (length >= sizeof(line_coding)) {
            for (uint32_t i = 0; i < sizeof(line_coding); i++) {
                line_coding[i] = pbuf[i];
            }
        }
        break;

    case CDC_GET_LINE_CODING:
        if (length >= sizeof(line_coding)) {
            for (uint32_t i = 0; i < sizeof(line_coding); i++) {
                pbuf[i] = line_coding[i];
            }
        }
        break;

    default:
        /* остальные запросы (DTR/RTS, break и т.п.) игнорируем */
        break;
    }
    return USBD_OK;
}

/** Приём из COM-порта: в этом тесте не используется, просто перевзводим приём. */
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len)
{
    (void)Len;
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, pbuf);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}

/** Передача не идёт: буфер UserTxBufferFS можно переписывать. */
static int cdc_tx_idle(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    return (hcdc != NULL) && (hcdc->TxState == 0);
}

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    if (!cdc_tx_idle()) {
        return USBD_BUSY;   /* предыдущая посылка ещё не ушла */
    }

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
    return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

int usb_cdc_ready(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1 : 0;
}

int usb_cdc_write(const uint8_t *data, uint16_t len)
{
    uint32_t started;
    uint16_t sent = 0;

    if (!usb_cdc_ready()) {
        return -1;      /* кабель не воткнут / хост не перечислил устройство */
    }

    started = HAL_GetTick();
    while (sent < len) {
        uint16_t chunk;

        /* Ждём, пока предыдущая посылка уйдёт: только тогда можно трогать буфер. */
        if (!cdc_tx_idle()) {
            if ((HAL_GetTick() - started) > CDC_TX_TIMEOUT_MS) {
                return -1;  /* порт на ПК не открыт — не ждём, идём опрашивать датчик */
            }
            continue;
        }

        chunk = (uint16_t)(len - sent);
        if (chunk > APP_TX_DATA_SIZE) {
            chunk = APP_TX_DATA_SIZE;
        }

        /* копируем в буфер класса: исходный может быть перезаписан вызывающей
           стороной до того, как USB закончит передачу */
        for (uint16_t i = 0; i < chunk; i++) {
            UserTxBufferFS[i] = data[sent + i];
        }

        if (CDC_Transmit_FS(UserTxBufferFS, chunk) != USBD_OK) {
            return -1;
        }
        sent = (uint16_t)(sent + chunk);
        started = HAL_GetTick();
    }

    /* Посылка, кратная размеру пакета, должна закрываться нулевым пакетом,
       иначе хост придержит данные до следующей передачи. */
    if ((len % APP_TX_DATA_SIZE) == 0U) {
        while (!cdc_tx_idle()) {
            if ((HAL_GetTick() - started) > CDC_TX_TIMEOUT_MS) {
                return (int)len;
            }
        }
        (void)CDC_Transmit_FS(UserTxBufferFS, 0);
    }

    return (int)len;
}
