/**
 * @file  usbd_conf.h
 * @brief Конфигурация USB Device Library (обычно генерируется CubeMX).
 */
#ifndef USBD_CONF_H
#define USBD_CONF_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

/* --- Параметры стека --- */
#define USBD_MAX_NUM_INTERFACES        2U    /* CDC: управляющий + данные */
#define USBD_MAX_NUM_CONFIGURATION     1U
#define USBD_MAX_STR_DESC_SIZ          128U
#define USBD_SUPPORT_USER_STRING       0U
#define USBD_SUPPORT_USER_STRING_DESC  0U
#define USBD_SELF_POWERED              0U    /* питание от шины USB */
#define USBD_DEBUG_LEVEL               0U
#define USBD_LPM_ENABLED               0U
#define USBD_CDC_INTERVAL              1000U

/* --- Память: статические буферы вместо malloc --- */
void *USBD_static_malloc(uint32_t size);
void  USBD_static_free(void *p);

#define USBD_malloc   (void *)USBD_static_malloc
#define USBD_free     USBD_static_free
#define USBD_memset   memset
#define USBD_memcpy   memcpy
#define USBD_Delay    HAL_Delay

/* --- Логи стека отключены --- */
#define USBD_UsrLog(...)  do { } while (0)
#define USBD_ErrLog(...)  do { } while (0)
#define USBD_DbgLog(...)  do { } while (0)

#endif /* USBD_CONF_H */
