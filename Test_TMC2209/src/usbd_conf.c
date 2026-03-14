/* usbd_conf.c — Привязка USB Device Library к USB OTG FS (STM32F446). */

#include "usbd_core.h"
#include "usbd_cdc.h"
#include "board.h"

#if defined(USB_OTG_FS)

#include "stm32f4xx_hal_pcd.h"

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* --- HAL PCD MSP --- */

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance == USB_OTG_FS) {
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA11 = D-, PA12 = D+ */
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = GPIO_PIN_11 | GPIO_PIN_12;
        gpio.Mode  = GPIO_MODE_AF_PP;
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF10_OTG_FS;
        HAL_GPIO_Init(GPIOA, &gpio);

        HAL_NVIC_SetPriority(OTG_FS_IRQn, IRQ_PRIO_USB, 0);
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance == USB_OTG_FS) {
        __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
        HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    }
}

/* --- PCD колбэки → USBD --- */

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData,
                        (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData,
                          epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData,
                         epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);
    USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}

/* --- USBD_LL (низкоуровневый интерфейс) --- */

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    hpcd_USB_OTG_FS.Instance                     = USB_OTG_FS;
    hpcd_USB_OTG_FS.Init.dev_endpoints           = 4;
    hpcd_USB_OTG_FS.Init.Host_channels           = 0;
    hpcd_USB_OTG_FS.Init.speed                   = PCD_SPEED_FULL;
    hpcd_USB_OTG_FS.Init.dma_enable               = DISABLE;
    hpcd_USB_OTG_FS.Init.ep0_mps                 = EP_MPS_64;
    hpcd_USB_OTG_FS.Init.phy_itface              = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG_FS.Init.Sof_enable              = DISABLE;
    hpcd_USB_OTG_FS.Init.low_power_enable         = DISABLE;
    hpcd_USB_OTG_FS.Init.lpm_enable               = DISABLE;
    hpcd_USB_OTG_FS.Init.battery_charging_enable  = DISABLE;
    hpcd_USB_OTG_FS.Init.vbus_sensing_enable      = DISABLE;
    hpcd_USB_OTG_FS.Init.use_dedicated_ep1       = DISABLE;
    hpcd_USB_OTG_FS.Init.use_external_vbus       = DISABLE;

    hpcd_USB_OTG_FS.pData = pdev;
    pdev->pData           = &hpcd_USB_OTG_FS;

    if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
        return USBD_FAIL;

    /* OTG FS FIFO RAM: 320 x 32-bit слов (1280 байт) */
    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 128);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 32);   /* EP0 TX */
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 128);  /* CDC data IN (EP1) */
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 32);   /* CDC CMD (EP2) */

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_DeInit(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Start(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Stop(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep, uint8_t type, uint16_t mps)
{
    HAL_PCD_EP_Open(pdev->pData, ep, mps, type);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    HAL_PCD_EP_Close(pdev->pData, ep);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    HAL_PCD_EP_Flush(pdev->pData, ep);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    HAL_PCD_EP_SetStall(pdev->pData, ep);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    HAL_PCD_EP_ClrStall(pdev->pData, ep);
    return USBD_OK;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    PCD_HandleTypeDef *hpcd = pdev->pData;
    if ((ep & 0x80U) != 0U)
        return hpcd->IN_ep[ep & 0x7FU].is_stall;
    else
        return hpcd->OUT_ep[ep].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t addr)
{
    HAL_PCD_SetAddress(pdev->pData, addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                     uint8_t *pbuf, uint32_t size)
{
    HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                           uint8_t *pbuf, uint32_t size)
{
    HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep)
{
    return HAL_PCD_EP_GetRxCount(pdev->pData, ep);
}

void USBD_LL_Delay(uint32_t Delay)
{
    Delay_ms(Delay);
}

/* --- USB IRQ --- */

void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

#endif /* USB_OTG_FS */
