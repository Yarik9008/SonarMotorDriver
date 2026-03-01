/* uart.c — UART (USART1, PA9 TX / PA10 RX) через DMA. */

#include "uart.h"
#include "board.h"

static UART_HandleTypeDef huart;
static DMA_HandleTypeDef  hdma_tx;
static DMA_HandleTypeDef  hdma_rx;

/* --- RX: DMA circular → буфер, позиция записи по NDTR --- */

static uint8_t  rx_buf[UART_RX_RING_SIZE];
static uint16_t rx_tail = 0;

static uint16_t rx_get_head(void)
{
    return UART_RX_RING_SIZE - __HAL_DMA_GET_COUNTER(huart.hdmarx);
}

/* --- TX: кольцевой буфер → DMA normal --- */

static uint8_t  tx_ring[UART_TX_RING_SIZE];
static volatile uint16_t tx_head    = 0;
static volatile uint16_t tx_tail    = 0;
static volatile uint16_t tx_pending = 0;

static uint16_t tx_get_count(void)
{
    uint16_t h = tx_head, t = tx_tail;
    return (h >= t) ? (h - t) : (UART_TX_RING_SIZE - t + h);
}

static uint16_t tx_get_free(void)
{
    return (UART_TX_RING_SIZE - 1U) - tx_get_count();
}

/* --- Инициализация --- */

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

    HAL_UART_Receive_DMA(&huart, rx_buf, UART_RX_RING_SIZE);
}

/* --- HAL MSP (тактирование, GPIO, DMA, NVIC) --- */

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    if (h->Instance != UART_INSTANCE)
        return;

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = UART_TX_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UART_TX_PORT, &gpio);

    gpio.Pin  = UART_RX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(UART_RX_PORT, &gpio);

    /* DMA1_Channel4 — USART1_TX (normal) */
    hdma_tx.Instance                 = DMA1_Channel4;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_tx);
    __HAL_LINKDMA(h, hdmatx, hdma_tx);

    /* DMA1_Channel5 — USART1_RX (circular) */
    hdma_rx.Instance                 = DMA1_Channel5;
    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_rx.Init.Priority            = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_rx);
    __HAL_LINKDMA(h, hdmarx, hdma_rx);

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, IRQ_PRIO_UART, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, IRQ_PRIO_UART, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    /* USART1 IRQ нужен для завершения DMA TX (TC флаг UART) */
    HAL_NVIC_SetPriority(USART1_IRQn, IRQ_PRIO_UART, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *h)
{
    if (h->Instance != UART_INSTANCE)
        return;

    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(UART_TX_PORT, UART_TX_PIN);
    HAL_GPIO_DeInit(UART_RX_PORT, UART_RX_PIN);
    HAL_DMA_DeInit(h->hdmatx);
    HAL_DMA_DeInit(h->hdmarx);
    HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
}

/* --- Обработчики прерываний --- */

void DMA1_Channel4_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_tx); }
void DMA1_Channel5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_rx); }
void USART1_IRQHandler(void)        { HAL_UART_IRQHandler(&huart);  }

/* --- Колбэк завершения DMA TX --- */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance != UART_INSTANCE)
        return;

    tx_tail = (tx_tail + tx_pending) % UART_TX_RING_SIZE;
    tx_pending = 0;
}

/* --- Публичный API --- */

uint8_t UART_Transmit(const uint8_t *buf, uint16_t len)
{
    if (tx_get_free() < len)
        return 1;

    for (uint16_t i = 0; i < len; i++) {
        tx_ring[tx_head] = buf[i];
        tx_head = (tx_head + 1) % UART_TX_RING_SIZE;
    }
    return 0;
}

void UART_Task(void)
{
    if (tx_pending != 0 || huart.gState != HAL_UART_STATE_READY)
        return;

    uint16_t count = tx_get_count();
    if (count == 0)
        return;

    uint16_t chunk = count;
    if (tx_tail + count > UART_TX_RING_SIZE)
        chunk = UART_TX_RING_SIZE - tx_tail;

    tx_pending = chunk;
    HAL_UART_Transmit_DMA(&huart, &tx_ring[tx_tail], chunk);
}

uint16_t UART_ReadLine(char *buf, uint16_t size)
{
    if (size == 0)
        return 0;

    uint16_t head = rx_get_head();
    uint16_t len  = 0;
    uint16_t tail = rx_tail;

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
