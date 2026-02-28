/**
 * @file usb_cdc.c
 * @brief Реализация USB CDC (виртуальный COM-порт).
 *
 * Этот файл связывает ST USB Device Library с конкретным приложением:
 *   - Регистрирует класс CDC и его колбэки (Init, Receive, Control).
 *   - Обрабатывает команды хоста (SET/GET_LINE_CODING — скорость, формат).
 *   - Предоставляет неблокирующую функцию передачи USB_CDC_Transmit()
 *     с использованием кольцевого буфера.
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
#include "board.h"

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

/* ======== Кольцевой буфер передачи ====================================== */

static uint8_t  tx_ring[USB_TX_RING_SIZE];
static uint16_t tx_head = 0;  /* Индекс записи */
static uint16_t tx_tail = 0;  /* Индекс чтения */

/** Вычисление количества занятых байт в буфере */
static uint16_t get_ring_count(void)
{
    if (tx_head >= tx_tail)
        return tx_head - tx_tail;
    else
        return USB_TX_RING_SIZE - tx_tail + tx_head;
}

/** Вычисление свободного места в буфере */
static uint16_t get_ring_free(void)
{
    return (USB_TX_RING_SIZE - 1) - get_ring_count();
}

/* ======== Колбэки интерфейса CDC ======================================= */

/** @brief Вызывается при подключении хоста — настраиваем буфер приёма. */
static int8_t CDC_Init_FS(void)
{
    tx_head = 0;
    tx_tail = 0;
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
 */
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

/**
 * @brief Вызывается при получении данных от хоста.
 * Распознаёт команду "DFU" — перезагрузка в DFU-загрузчик.
 */
static int8_t CDC_Receive_FS(uint8_t *buf, uint32_t *len)
{
    if (*len >= 3 && buf[0] == 'D' && buf[1] == 'F' && buf[2] == 'U')
        USB_CDC_RebootToDFU();

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}

/** Таблица колбэков CDC-интерфейса */
static USBD_CDC_ItfTypeDef cdc_fops = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
};

/* ======== Публичный API ================================================= */

/**
 * @brief Инициализация USB CDC.
 */
void USB_CDC_Init(void)
{
    tx_head = 0;
    tx_tail = 0;
    USBD_Init(&hUsbDeviceFS, &CDC_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &cdc_fops);
    USBD_Start(&hUsbDeviceFS);
}

/**
 * @brief Отправка данных в кольцевой буфер USB CDC.
 * @return 0 если данные успешно помещены в буфер, 1 если места не хватило.
 */
uint8_t USB_CDC_Transmit(const uint8_t *buf, uint16_t len)
{
    if (get_ring_free() < len) {
        return 1; /* Переполнение буфера */
    }

    uint16_t i;
    for (i = 0; i < len; i++) {
        tx_ring[tx_head] = buf[i];
        tx_head = (tx_head + 1) % USB_TX_RING_SIZE;
    }

    return 0;
}

/**
 * @brief Фоновая задача обработки передачи.
 *
 * Берёт данные из кольцевого буфера и отправляет их аппаратуре USB,
 * если предыдущая передача завершена.
 */
void USB_CDC_Task(void)
{
    if (!USB_CDC_IsConnected()) {
        /* Очищаем буфер, если кабель отключён, чтобы не копить старые данные */
        tx_head = tx_tail = 0;
        return;
    }

    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if (hcdc == NULL || hcdc->TxState != 0) {
        return; /* CDC не инициализирован или предыдущая отправка не завершена */
    }

    uint16_t count = get_ring_count();
    if (count == 0) {
        return; /* Нет данных для отправки */
    }

    /* Ограничиваем размер одного пакета (обычно до размера буфера конечной точки)
     * Но библиотека USBD может принимать больше и дробить сама.
     * Отправим сколько сможем линейно (до конца массива кольцевого буфера). */
    uint16_t chunk_size = count;
    if (tx_tail + count > USB_TX_RING_SIZE) {
        chunk_size = USB_TX_RING_SIZE - tx_tail; /* Читаем только до физического конца массива */
    }

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &tx_ring[tx_tail], chunk_size);
    if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK) {
        /* Сдвигаем хвост кольцевого буфера только после успешного старта передачи */
        tx_tail = (tx_tail + chunk_size) % USB_TX_RING_SIZE;
    }
}

/** @brief Проверка, подключён ли USB-хост и устройство в рабочем состоянии. */
uint8_t USB_CDC_IsConnected(void)
{
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

/**
 * @brief Перезагрузка в DFU-загрузчик.
 */
void USB_CDC_RebootToDFU(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    BKP->DR10 = 0x424C;

    NVIC_SystemReset();
}
