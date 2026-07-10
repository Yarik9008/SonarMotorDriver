/**
 * @file biss_port.h
 * @brief Слой абстракции платформы для библиотеки BiSS-C.
 */

#ifndef BISS_ENCODER_PORT_H
#define BISS_ENCODER_PORT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 0 = busy, 1 = complete OK, 2 = complete with error */
#define BISS_ASYNC_BUSY   0U
#define BISS_ASYNC_OK     1U
#define BISS_ASYNC_ERROR  2U

typedef struct {
    /**
     * Инициализация транспорта (SPI, RS-485 DE/RE).
     * Возврат: 0 = OK, иначе ошибка.
     */
    int (*init)(void *ctx);

    /**
     * Блокирующий SPI-обмен.
     * Возврат: 0 = OK, иначе ошибка.
     */
    int (*spi_xfer)(const uint8_t *tx, uint8_t *rx, size_t len,
                    uint32_t timeout_ms, void *ctx);

    /**
     * Неблокирующий SPI-обмен (DMA и т.п.).
     * Возврат: 0 = запущено, иначе ошибка.
     * Может быть NULL, если async не поддерживается.
     */
    int (*spi_xfer_async_start)(const uint8_t *tx, uint8_t *rx, size_t len,
                                void *ctx);

    /**
     * Статус async-обмена.
     * Возврат: BISS_ASYNC_BUSY / BISS_ASYNC_OK / BISS_ASYNC_ERROR.
     */
    uint8_t (*spi_xfer_async_poll)(void *ctx);

    /**
     * Аварийное прерывание зависшего async-обмена (безопасно из ISR).
     */
    void (*spi_abort)(void *ctx);

    void *ctx;
} biss_port_t;

#ifdef __cplusplus
}
#endif

#endif /* BISS_ENCODER_PORT_H */
