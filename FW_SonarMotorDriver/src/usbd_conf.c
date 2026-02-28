/**
 * @file usbd_conf.c
 * @brief Низкоуровневая привязка USB Device Library к аппаратуре STM32F103.
 *
 * Этот файл — «мост» между ST USB Device Library (аппаратно-независимой)
 * и конкретным микроконтроллером. Содержит:
 *
 *   1. HAL PCD MSP Init/DeInit — инициализация тактирования, GPIO и прерываний USB.
 *   2. PCD колбэки — перенаправляют события от HAL PCD в ядро USBD.
 *   3. USBD_LL_* функции — реализация низкоуровневого интерфейса, который
 *      вызывает ядро USB Device Library для управления эндпоинтами.
 *   4. Обработчик прерывания USB.
 *
 * PMA (Packet Memory Area) — специальная память USB-периферии STM32F103 (512 байт).
 * Каждому эндпоинту выделяется область в PMA для буферизации пакетов.
 */

#include "usbd_core.h"
#include "usbd_cdc.h"
#include "board.h"

/** Хэндл PCD (Peripheral Controller Driver) — аппаратный уровень USB */
PCD_HandleTypeDef hpcd_USB_FS;

/* ======== HAL PCD MSP (инициализация аппаратуры USB) ==================== */

/**
 * @brief Инициализация аппаратных ресурсов USB.
 *
 * Вызывается автоматически из HAL_PCD_Init().
 * Делает:
 *   1. Включает тактирование USB-периферии.
 *   2. Кратковременно притягивает PA12 (USB DP) к земле — это заставляет
 *      хост заново определить устройство (ре-энумерация).
 *   3. Настраивает прерывание USB с приоритетом 5.
 */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance == USB) {
        __HAL_RCC_USB_CLK_ENABLE();

        /* Ре-энумерация: притягиваем DP к земле на 5 мс,
         * чтобы хост увидел «отключение» и заново определил устройство */
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = GPIO_PIN_12;
        gpio.Mode  = GPIO_MODE_OUTPUT_PP;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &gpio);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_Delay(5);

        /* Возвращаем PA11 (DM) и PA12 (DP) в режим входа — USB-периферия
         * сама управляет этими пинами через аналоговый трансивер */
        gpio.Pin  = GPIO_PIN_11 | GPIO_PIN_12;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio);

        /* Прерывание USB LP — обрабатывает все USB-события */
        HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, IRQ_PRIO_USB, 0);
        HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    }
}

/** @brief Деинициализация аппаратных ресурсов USB. */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance == USB) {
        __HAL_RCC_USB_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    }
}

/* ======== PCD колбэки -> ядро USBD ====================================== */
/* HAL PCD вызывает эти функции при USB-событиях.
 * Мы перенаправляем их в ядро USB Device Library (USBD_LL_*),
 * которое, в свою очередь, вызывает колбэки класса CDC. */

/** @brief Хост отправил SETUP-пакет (управляющий запрос). */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData,
                        (uint8_t *)hpcd->Setup);
}

/** @brief Завершён приём данных на эндпоинте (OUT — от хоста к устройству). */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData,
                          epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

/** @brief Завершена передача данных на эндпоинте (IN — от устройства к хосту). */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData,
                         epnum, hpcd->IN_ep[epnum].xfer_buff);
}

/** @brief Получен SOF (Start of Frame) — хост активен, шина работает. */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}

/** @brief Хост сбросил шину — устройство должно вернуться в начальное состояние. */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);
    USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

/** @brief Хост приостановил шину (Suspend). */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
}

/** @brief Хост возобновил работу шины (Resume). */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}

/* ======== Низкоуровневый интерфейс USBD (USBD_LL_*) ===================== */
/* Эти функции вызываются ядром USB Device Library.
 * Каждая из них — тонкая обёртка над HAL PCD API. */

/**
 * @brief Инициализация USB-периферии и разметка PMA.
 *
 * PMA (Packet Memory Area) — 512 байт встроенной памяти USB-контроллера.
 * Каждому эндпоинту нужен свой буфер в PMA:
 *   Адрес 0x18  — EP0 OUT (управляющий, приём от хоста)
 *   Адрес 0x58  — EP0 IN  (управляющий, передача хосту)
 *   Адрес 0x98  — CDC IN  (данные от устройства к хосту, наши строки с позицией)
 *   Адрес 0xD8  — CDC OUT (данные от хоста к устройству, входящие команды)
 *   Адрес 0x118 — CDC CMD (управляющие команды CDC, SET/GET_LINE_CODING)
 */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    hpcd_USB_FS.Instance                 = USB;
    hpcd_USB_FS.Init.dev_endpoints       = 8;
    hpcd_USB_FS.Init.speed               = PCD_SPEED_FULL;    /* USB Full Speed (12 Мбит/с) */
    hpcd_USB_FS.Init.ep0_mps             = EP_MPS_64;         /* EP0 макс. пакет = 64 байта */
    hpcd_USB_FS.Init.low_power_enable    = DISABLE;
    hpcd_USB_FS.Init.lpm_enable          = DISABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;

    /* Перекрёстная привязка: PCD знает про USBD и наоборот */
    hpcd_USB_FS.pData = pdev;
    pdev->pData       = &hpcd_USB_FS;

    if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
        return USBD_FAIL;

    /* Разметка PMA: каждому эндпоинту назначаем адрес и тип буфера */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00, PCD_SNG_BUF, 0x18);     /* EP0 OUT */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80, PCD_SNG_BUF, 0x58);     /* EP0 IN  */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_IN_EP,  PCD_SNG_BUF, 0x98);  /* CDC данные IN  */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_OUT_EP, PCD_SNG_BUF, 0xD8);  /* CDC данные OUT */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, CDC_CMD_EP, PCD_SNG_BUF, 0x118); /* CDC команды */

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_DeInit(pdev->pData);
    return USBD_OK;
}

/** @brief Запуск USB — подключение подтяжки DP, хост начнёт энумерацию. */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Start(pdev->pData);
    return USBD_OK;
}

/** @brief Остановка USB — устройство отключается от шины. */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Stop(pdev->pData);
    return USBD_OK;
}

/** @brief Открытие эндпоинта (задаёт тип и максимальный размер пакета). */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep, uint8_t type, uint16_t mps)
{
    HAL_PCD_EP_Open(pdev->pData, ep, mps, type);
    return USBD_OK;
}

/** @brief Закрытие эндпоинта. */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    HAL_PCD_EP_Close(pdev->pData, ep);
    return USBD_OK;
}

/** @brief Сброс буфера эндпоинта. */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    HAL_PCD_EP_Flush(pdev->pData, ep);
    return USBD_OK;
}

/** @brief Установка STALL на эндпоинте (сигнал ошибки хосту). */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    HAL_PCD_EP_SetStall(pdev->pData, ep);
    return USBD_OK;
}

/** @brief Снятие STALL с эндпоинта. */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    HAL_PCD_EP_ClrStall(pdev->pData, ep);
    return USBD_OK;
}

/** @brief Проверка, установлен ли STALL на эндпоинте. */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    PCD_HandleTypeDef *hpcd = pdev->pData;
    if ((ep & 0x80) != 0)
        return hpcd->IN_ep[ep & 0x7F].is_stall;
    else
        return hpcd->OUT_ep[ep].is_stall;
}

/** @brief Установка USB-адреса устройства (назначается хостом при энумерации). */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t addr)
{
    HAL_PCD_SetAddress(pdev->pData, addr);
    return USBD_OK;
}

/** @brief Передача данных хосту через IN-эндпоинт. */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep,
                                     uint8_t *pbuf, uint16_t size)
{
    HAL_PCD_EP_Transmit(pdev->pData, ep, pbuf, size);
    return USBD_OK;
}

/** @brief Подготовка приёма данных от хоста на OUT-эндпоинте. */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep,
                                           uint8_t *pbuf, uint16_t size)
{
    HAL_PCD_EP_Receive(pdev->pData, ep, pbuf, size);
    return USBD_OK;
}

/** @brief Получить количество принятых байт на OUT-эндпоинте. */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    return HAL_PCD_EP_GetRxCount(pdev->pData, ep);
}

/** @brief Задержка в миллисекундах (используется ядром USB при инициализации). */
void USBD_LL_Delay(uint32_t Delay)
{
    HAL_Delay(Delay);
}

/* ======== Обработчик прерывания USB ====================================== */

/**
 * @brief Прерывание USB LP (Low Priority).
 *
 * На STM32F103 USB и CAN1 разделяют один вектор прерывания.
 * HAL_PCD_IRQHandler разбирает флаги и вызывает соответствующие колбэки выше.
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}
