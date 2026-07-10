/**
 * @file biss_port_stm32_hal.h
 * @brief Порт BiSS-C для STM32Cube HAL (SPI + DMA).
 */

#ifndef BISS_ENCODER_PORT_STM32_HAL_H
#define BISS_ENCODER_PORT_STM32_HAL_H

#include "biss_encoder/biss_port.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    SPI_HandleTypeDef  hspi;
    DMA_HandleTypeDef  hdma_rx;
    DMA_HandleTypeDef  hdma_tx;
    GPIO_TypeDef      *de_port;
    uint16_t            de_pin;
    GPIO_TypeDef      *re_port;
    uint16_t            re_pin;
    uint8_t             use_dma;
    uint8_t             dma_irq_prio;
    volatile uint8_t    dma_done;
    volatile uint8_t    dma_error;
} biss_hal_ctx_t;

void biss_port_stm32_hal_fill(biss_port_t *port, biss_hal_ctx_t *ctx);

void biss_port_stm32_hal_spi_msp_init(SPI_HandleTypeDef *hspi, biss_hal_ctx_t *ctx);
void biss_port_stm32_hal_spi_msp_deinit(SPI_HandleTypeDef *hspi, biss_hal_ctx_t *ctx);

void biss_port_stm32_hal_on_spi_complete(SPI_HandleTypeDef *hspi, biss_hal_ctx_t *ctx);
void biss_port_stm32_hal_on_spi_error(SPI_HandleTypeDef *hspi, biss_hal_ctx_t *ctx);

void biss_port_stm32_hal_dma_rx_irq(biss_hal_ctx_t *ctx);
void biss_port_stm32_hal_dma_tx_irq(biss_hal_ctx_t *ctx);

biss_hal_ctx_t *biss_port_stm32_hal_get_active_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* BISS_ENCODER_PORT_STM32_HAL_H */
