/* usb_cdc.c — USB CDC (виртуальный COM-порт): колбэки, кольцевые буферы TX/RX. */

#include "usb_cdc.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "board.h"

USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t cdc_rx_buf[CDC_DATA_FS_MAX_PACKET_SIZE];

static uint8_t  rx_ring[USB_RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static uint16_t rx_tail = 0;

static USBD_CDC_LineCodingTypeDef line_coding = {
    .bitrate    = 115200,
    .format     = 0x00, /* 1 стоп-бит   */
    .paritytype = 0x00, /* Без чётности  */
    .datatype   = 0x08, /* 8 бит данных  */
};

/* --- Кольцевой буфер передачи --- */

static uint8_t  tx_ring[USB_TX_RING_SIZE];
static uint16_t tx_head = 0;
static uint16_t tx_tail = 0;

static uint16_t get_ring_count(void)
{
    if (tx_head >= tx_tail)
        return tx_head - tx_tail;
    else
        return USB_TX_RING_SIZE - tx_tail + tx_head;
}

static uint16_t get_ring_free(void)
{
    return (USB_TX_RING_SIZE - 1) - get_ring_count();
}

/* --- Колбэки CDC --- */

static int8_t CDC_Init_FS(void)
{
    tx_head = 0;
    tx_tail = 0;
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf);
    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    return USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)length;
    switch (cmd) {
    case CDC_SET_LINE_CODING:
        line_coding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |
                                            (pbuf[2] << 16) | (pbuf[3] << 24));
        line_coding.format     = pbuf[4];
        line_coding.paritytype = pbuf[5];
        line_coding.datatype   = pbuf[6];
        break;
    case CDC_GET_LINE_CODING:
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

static int8_t CDC_Receive_FS(uint8_t *buf, uint32_t *len)
{
    if (*len >= 3 && buf[0] == 'D' && buf[1] == 'F' && buf[2] == 'U')
        USB_CDC_RebootToDFU();

    for (uint32_t i = 0; i < *len; i++) {
        uint16_t next = (rx_head + 1) % USB_RX_RING_SIZE;
        if (next != rx_tail) {
            rx_ring[rx_head] = buf[i];
            rx_head = next;
        }
    }

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}

static USBD_CDC_ItfTypeDef cdc_fops = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
};

/* --- Публичный API --- */

void USB_CDC_Init(void)
{
    tx_head = 0;
    tx_tail = 0;
    USBD_Init(&hUsbDeviceFS, &CDC_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &cdc_fops);
    USBD_Start(&hUsbDeviceFS);
}

uint8_t USB_CDC_Transmit(const uint8_t *buf, uint16_t len)
{
    if (get_ring_free() < len)
        return 1;

    uint16_t i;
    for (i = 0; i < len; i++) {
        tx_ring[tx_head] = buf[i];
        tx_head = (tx_head + 1) % USB_TX_RING_SIZE;
    }

    return 0;
}

void USB_CDC_Task(void)
{
    if (!USB_CDC_IsConnected()) {
        tx_head = tx_tail = 0;
        return;
    }

    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if (hcdc == NULL || hcdc->TxState != 0)
        return;

    uint16_t count = get_ring_count();
    if (count == 0)
        return;

    uint16_t chunk_size = count;
    if (tx_tail + count > USB_TX_RING_SIZE)
        chunk_size = USB_TX_RING_SIZE - tx_tail;

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &tx_ring[tx_tail], chunk_size);
    if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK)
        tx_tail = (tx_tail + chunk_size) % USB_TX_RING_SIZE;
}

uint8_t USB_CDC_IsConnected(void)
{
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

uint16_t USB_CDC_ReadLine(char *buf, uint16_t size)
{
    if (size == 0)
        return 0;

    uint16_t len = 0;
    uint16_t tail = rx_tail;
    uint8_t found_eol = 0;

    while (tail != rx_head) {
        uint8_t c = rx_ring[tail];
        tail = (tail + 1) % USB_RX_RING_SIZE;

        if (c == '\r' || c == '\n') {
            if (c == '\r' && tail != rx_head && rx_ring[tail] == '\n')
                tail = (tail + 1) % USB_RX_RING_SIZE;
            found_eol = 1;
            break;
        }

        if (len < size - 1)
            buf[len++] = (char)c;
    }

    if (!found_eol)
        return 0;

    buf[len] = '\0';
    rx_tail = tail;
    return len;
}

void USB_CDC_RebootToDFU(void)
{
    /* F4: нет BKP. Для входа в DFU системного загрузчика: BOOT0=1 и сброс. */
    NVIC_SystemReset();
}
