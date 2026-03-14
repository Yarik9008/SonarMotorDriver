/* uart.c — UART (USART3, PB10 TX / PB11 RX) через прерывания.
 * PA9/PA10 зарезервированы для UART bootloader.
 *
 * USART2 (TMC2209) настраивается здесь через HAL_UART_MspInit, но
 * используется только библиотекой TMC2209 в blocking-режиме (без IRQ).
 */

#include "uart.h"
#include "board.h"

static UART_HandleTypeDef huart;

static uint8_t  rx_buf[UART_RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static uint16_t rx_tail = 0;
static uint8_t  rx_byte;   /* Буфер для приёма по 1 байту */

static uint16_t rx_get_head(void)
{
    return rx_head;
}

static uint8_t  tx_ring[UART_TX_RING_SIZE];
static volatile uint16_t tx_head    = 0;
static volatile uint16_t tx_tail    = 0;
static volatile uint16_t tx_pending = 0;

static uint16_t tx_count(void)
{
    uint16_t h = tx_head, t = tx_tail;
    return (h >= t) ? (h - t) : (UART_TX_RING_SIZE - t + h);
}

static uint16_t tx_free(void)
{
    return (UART_TX_RING_SIZE - 1U) - tx_count();
}

static void uart_rx_restart(void)
{
    HAL_UART_Receive_IT(&huart, &rx_byte, 1);
}

void UART_Init(void)
{
    huart.Instance          = UART_INSTANCE;
    huart.Init.BaudRate     = UART_BAUDRATE;
    huart.Init.WordLength   = UART_WORDLENGTH_8B;
    huart.Init.StopBits     = UART_STOPBITS_1;
    huart.Init.Parity       = UART_PARITY_NONE;
    huart.Init.Mode         = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart);
    uart_rx_restart();
}

/* --- HAL MSP --- */

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    if (h->Instance == TMC2209_UART) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = TMC2209_UART_TX_PIN; gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(TMC2209_UART_TX_PORT, &gpio);
        gpio.Pin = TMC2209_UART_RX_PIN; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(TMC2209_UART_RX_PORT, &gpio);
        return;
    }
    if (h->Instance != UART_INSTANCE) return;

    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = UART_TX_PIN; gpio.Mode = GPIO_MODE_AF_PP; gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UART_TX_PORT, &gpio);
    gpio.Pin = UART_RX_PIN; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(UART_RX_PORT, &gpio);

    HAL_NVIC_SetPriority(USART3_IRQn, IRQ_PRIO_UART, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *h)
{
    if (h->Instance == TMC2209_UART) {
        __HAL_RCC_USART2_CLK_DISABLE();
        HAL_GPIO_DeInit(TMC2209_UART_TX_PORT, TMC2209_UART_TX_PIN);
        HAL_GPIO_DeInit(TMC2209_UART_RX_PORT, TMC2209_UART_RX_PIN);
        return;
    }
    if (h->Instance != UART_INSTANCE) return;
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(UART_TX_PORT, UART_TX_PIN);
    HAL_GPIO_DeInit(UART_RX_PORT, UART_RX_PIN);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
}

/* --- IRQ --- */

void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart); }

/* --- HAL колбэки (только USART3 — командный интерфейс) --- */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance != UART_INSTANCE) return;
    tx_tail = (tx_tail + tx_pending) % UART_TX_RING_SIZE;
    tx_pending = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance != UART_INSTANCE) return;
    uint16_t next = (rx_head + 1) % UART_RX_RING_SIZE;
    if (next != rx_tail) {
        rx_buf[rx_head] = rx_byte;
        rx_head = next;
    }
    uart_rx_restart();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == UART_INSTANCE) uart_rx_restart();
}

/* --- API --- */

uint8_t UART_Transmit(const uint8_t *buf, uint16_t len)
{
    if (tx_free() < len) return 1;
    for (uint16_t i = 0; i < len; i++) {
        tx_ring[tx_head] = buf[i];
        tx_head = (tx_head + 1) % UART_TX_RING_SIZE;
    }
    return 0;
}

void UART_Task(void)
{
    if (tx_pending || huart.gState != HAL_UART_STATE_READY) return;
    uint16_t cnt = tx_count();
    if (!cnt) return;
    uint16_t chunk = (tx_tail + cnt > UART_TX_RING_SIZE) ? (UART_TX_RING_SIZE - tx_tail) : cnt;
    tx_pending = chunk;
    HAL_UART_Transmit_IT(&huart, &tx_ring[tx_tail], chunk);
}

uint16_t UART_ReadLine(char *buf, uint16_t size)
{
    if (!size) return 0;
    uint16_t head = rx_get_head(), len = 0, tail = rx_tail;
    while (tail != head && len < size - 1) {
        uint8_t c = rx_buf[tail];
        tail = (tail + 1) % UART_RX_RING_SIZE;
        if (c == '\r' || c == '\n') {
            rx_tail = tail;
            buf[len] = '\0';
            return len;
        }
        buf[len++] = (char)c;
    }
    return 0;
}
