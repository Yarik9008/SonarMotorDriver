/**
 * @file uart.c
 * @brief Реализация командного UART (USART1 + USART3).
 *
 * USART1 (PA9/PA10) — DMA. USART3 (PB10/PB11) — DMA или IT (см. UART3_USE_DMA).
 * TX дублируется на оба порта, RX объединяется в общий буфер.
 */

#include "uart.h"
#include "board.h"
#include "line_reader.h"

typedef struct {
    USART_TypeDef       *instance;
    UART_HandleTypeDef   huart;
    DMA_HandleTypeDef    hdma_rx;
    DMA_HandleTypeDef    hdma_tx;
    GPIO_TypeDef        *tx_port;
    uint16_t             tx_pin;
    GPIO_TypeDef        *rx_port;
    uint16_t             rx_pin;
    uint8_t              use_dma;
    uint8_t              rx_it_byte;
    uint8_t              dma_rx_buf[UART_RX_DMA_SIZE];
    volatile uint16_t    dma_rx_prev;
    uint8_t              tx_ring[UART_TX_RING_SIZE];
    volatile uint16_t    tx_head;
    volatile uint16_t    tx_tail;
    volatile uint8_t     tx_busy;
    volatile uint8_t     ready;
    volatile uint8_t     msp_error;
} UartPortCtx;

static UartPortCtx s_ports[UART_PORT_COUNT];

static uint8_t           rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static uint16_t          rx_tail = 0;
static volatile uint8_t  s_uart_ready = 0;

static UartPortCtx *uart_ctx_from_handle(UART_HandleTypeDef *h)
{
    for (uint8_t i = 0; i < UART_PORT_COUNT; i++) {
        if (h == &s_ports[i].huart)
            return &s_ports[i];
    }
    return NULL;
}

static UartPortCtx *uart_ctx_from_instance(USART_TypeDef *inst)
{
    for (uint8_t i = 0; i < UART_PORT_COUNT; i++) {
        if (inst == s_ports[i].instance)
            return &s_ports[i];
    }
    return NULL;
}

static uint16_t tx_count(const UartPortCtx *p)
{
    uint16_t h = p->tx_head, t = p->tx_tail;
    return (h >= t) ? (h - t) : (UART_TX_RING_SIZE - t + h);
}

static uint16_t tx_free(const UartPortCtx *p)
{
    return (UART_TX_RING_SIZE - 1U) - tx_count(p);
}

static void uart_rx_push_byte(uint8_t c)
{
    uint16_t next = (rx_head + 1U) % UART_RX_RING_SIZE;
    if (next != rx_tail) {
        rx_ring[rx_head] = c;
        rx_head = next;
    }
}

static void uart_rx_drain(UartPortCtx *p)
{
    uint16_t write_pos = UART_RX_DMA_SIZE -
                         (uint16_t)__HAL_DMA_GET_COUNTER(&p->hdma_rx);

    uint16_t prev = p->dma_rx_prev;
    if (write_pos == prev)
        return;

    while (prev != write_pos) {
        uart_rx_push_byte(p->dma_rx_buf[prev]);
        prev = (prev + 1U) % UART_RX_DMA_SIZE;
    }

    p->dma_rx_prev = write_pos;
}

static void uart_port_reset(UartPortCtx *p)
{
    p->dma_rx_prev = 0;
    p->tx_head = 0;
    p->tx_tail = 0;
    p->tx_busy = 0;
    p->ready = 0;
    p->msp_error = 0;
    p->use_dma = 0;
    p->rx_it_byte = 0;
}

static void uart_reset_runtime_state(void)
{
    rx_head = 0;
    rx_tail = 0;
    for (uint8_t i = 0; i < UART_PORT_COUNT; i++)
        uart_port_reset(&s_ports[i]);
}

static void uart_port_rollback(UartPortCtx *p)
{
    /* Порт мог не дойти до HAL_UART_Init — деинициализировать нечего,
     * а HAL_UART_DeInit с нулевым Instance приводит к hard fault. */
    if (p->huart.Instance != NULL)
        HAL_UART_DeInit(&p->huart);
    uart_port_reset(p);
}

static void uart_port_rx_start(UartPortCtx *p)
{
    if (p->use_dma) {
        p->dma_rx_prev = 0;
        if (HAL_UART_Receive_DMA(&p->huart, p->dma_rx_buf, UART_RX_DMA_SIZE) == HAL_OK)
            __HAL_UART_ENABLE_IT(&p->huart, UART_IT_IDLE);
    } else {
        HAL_UART_Receive_IT(&p->huart, &p->rx_it_byte, 1);
    }
}

static void uart_port_msp_init(UartPortCtx *p)
{
    GPIO_InitTypeDef gpio = {0};

    if (p->instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (p->instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else {
        p->msp_error = 1;
        return;
    }

    gpio.Pin = p->tx_pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(p->tx_port, &gpio);
    gpio.Pin = p->rx_pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(p->rx_port, &gpio);

    if (p->use_dma) {
        __HAL_RCC_DMA1_CLK_ENABLE();

        if (p->instance == USART1) {
            p->hdma_rx.Instance = DMA1_Channel5;
            p->hdma_tx.Instance = DMA1_Channel4;
        } else {
            p->hdma_rx.Instance = DMA1_Channel3;
            p->hdma_tx.Instance = DMA1_Channel2;
        }

        p->hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        p->hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        p->hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        p->hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        p->hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        p->hdma_rx.Init.Mode                = DMA_CIRCULAR;
        p->hdma_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
        if (HAL_DMA_Init(&p->hdma_rx) != HAL_OK) {
            p->msp_error = 1;
            return;
        }
        __HAL_LINKDMA(&p->huart, hdmarx, p->hdma_rx);

        p->hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        p->hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        p->hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
        p->hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        p->hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        p->hdma_tx.Init.Mode                = DMA_NORMAL;
        p->hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&p->hdma_tx) != HAL_OK) {
            HAL_DMA_DeInit(&p->hdma_rx);
            p->huart.hdmarx = NULL;
            p->msp_error = 1;
            return;
        }
        __HAL_LINKDMA(&p->huart, hdmatx, p->hdma_tx);

        if (p->instance == USART1) {
            HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, IRQ_PRIO_UART, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
            HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, IRQ_PRIO_UART, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
        }
#if UART3_USE_DMA
        else {
            HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, IRQ_PRIO_UART, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
            HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, IRQ_PRIO_UART, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
        }
#endif
    }

    if (p->instance == USART1) {
        HAL_NVIC_SetPriority(USART1_IRQn, IRQ_PRIO_UART, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    } else {
        HAL_NVIC_SetPriority(USART3_IRQn, IRQ_PRIO_UART, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

static void uart_port_msp_deinit(UartPortCtx *p)
{
    if (p->use_dma) {
        HAL_DMA_DeInit(&p->hdma_rx);
        HAL_DMA_DeInit(&p->hdma_tx);
    }

    HAL_GPIO_DeInit(p->tx_port, p->tx_pin);
    HAL_GPIO_DeInit(p->rx_port, p->rx_pin);

    if (p->instance == USART1) {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
    } else if (p->instance == USART3) {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(USART3_IRQn);
#if UART3_USE_DMA
        HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
#endif
    }
}

static int uart_port_init(UartPortCtx *p, USART_TypeDef *instance, uint32_t baud,
                          GPIO_TypeDef *tx_port, uint16_t tx_pin,
                          GPIO_TypeDef *rx_port, uint16_t rx_pin,
                          uint8_t use_dma)
{
    uart_port_reset(p);
    p->instance = instance;
    p->tx_port = tx_port;
    p->tx_pin = tx_pin;
    p->rx_port = rx_port;
    p->rx_pin = rx_pin;
    p->use_dma = use_dma;

    p->huart.Instance          = instance;
    p->huart.Init.BaudRate     = baud;
    p->huart.Init.WordLength    = UART_WORDLENGTH_8B;
    p->huart.Init.StopBits      = UART_STOPBITS_1;
    p->huart.Init.Parity        = UART_PARITY_NONE;
    p->huart.Init.Mode          = UART_MODE_TX_RX;
    p->huart.Init.HwFlowCtl     = UART_HWCONTROL_NONE;
    p->huart.Init.OverSampling  = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&p->huart) != HAL_OK || p->msp_error)
        return -1;

    uart_port_rx_start(p);
    p->ready = 1;
    return 0;
}

static void uart_port_task(UartPortCtx *p)
{
    if (!p->ready || p->tx_busy)
        return;

    uint16_t cnt = tx_count(p);
    if (!cnt)
        return;

    uint16_t chunk = (p->tx_tail + cnt > UART_TX_RING_SIZE)
                     ? (UART_TX_RING_SIZE - p->tx_tail) : cnt;

    p->tx_busy = 1;
    uint16_t old_tail = p->tx_tail;
    p->tx_tail = (p->tx_tail + chunk) % UART_TX_RING_SIZE;

    HAL_StatusTypeDef st;
    if (p->use_dma)
        st = HAL_UART_Transmit_DMA(&p->huart, &p->tx_ring[old_tail], chunk);
    else
        st = HAL_UART_Transmit_IT(&p->huart, &p->tx_ring[old_tail], chunk);

    if (st != HAL_OK) {
        p->tx_tail = old_tail;
        p->tx_busy = 0;
    }
}

int UART_Init(void)
{
    uart_reset_runtime_state();
    s_uart_ready = 0;

    s_ports[0].instance = UART_INSTANCE;
    s_ports[1].instance = UART3_INSTANCE;

    if (uart_port_init(&s_ports[0], UART_INSTANCE, UART_BAUDRATE,
                       UART_TX_PORT, UART_TX_PIN, UART_RX_PORT, UART_RX_PIN, 1U) != 0) {
        uart_port_rollback(&s_ports[0]);
        uart_port_rollback(&s_ports[1]);
        return -1;
    }

    if (uart_port_init(&s_ports[1], UART3_INSTANCE, UART3_BAUDRATE,
                       UART3_TX_PORT, UART3_TX_PIN, UART3_RX_PORT, UART3_RX_PIN,
                       UART3_USE_DMA) != 0) {
        uart_port_rollback(&s_ports[1]);
    }

    s_uart_ready = 1;
    return 0;
}

void UART_CommandMspInit(UART_HandleTypeDef *h)
{
    UartPortCtx *p = uart_ctx_from_instance(h->Instance);
    if (p)
        uart_port_msp_init(p);
}

void UART_CommandMspDeInit(UART_HandleTypeDef *h)
{
    UartPortCtx *p = uart_ctx_from_instance(h->Instance);
    if (p)
        uart_port_msp_deinit(p);
}

void USART1_IRQHandler(void)
{
    UartPortCtx *p = &s_ports[0];
    if (p->use_dma && __HAL_UART_GET_FLAG(&p->huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&p->huart);
        uart_rx_drain(p);
    }
    HAL_UART_IRQHandler(&p->huart);
}

void USART3_IRQHandler(void)
{
    UartPortCtx *p = &s_ports[1];
    if (!p->ready)
        return;
    if (p->use_dma && __HAL_UART_GET_FLAG(&p->huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&p->huart);
        uart_rx_drain(p);
    }
    HAL_UART_IRQHandler(&p->huart);
}

void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_ports[0].hdma_tx);
}

void DMA1_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_ports[0].hdma_rx);
    uart_rx_drain(&s_ports[0]);
}

#if UART3_USE_DMA
void DMA1_Channel2_IRQHandler(void)
{
    if (s_ports[1].ready)
        HAL_DMA_IRQHandler(&s_ports[1].hdma_tx);
}

void DMA1_Channel3_IRQHandler(void)
{
    if (!s_ports[1].ready)
        return;
    HAL_DMA_IRQHandler(&s_ports[1].hdma_rx);
    uart_rx_drain(&s_ports[1]);
}
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    UartPortCtx *p = uart_ctx_from_handle(h);
    if (p)
        p->tx_busy = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    UartPortCtx *p = uart_ctx_from_handle(h);
    if (!p || !p->ready || p->use_dma)
        return;

    uart_rx_push_byte(p->rx_it_byte);
    HAL_UART_Receive_IT(h, &p->rx_it_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    UartPortCtx *p = uart_ctx_from_handle(h);
    if (!p || !p->ready)
        return;

    __HAL_UART_CLEAR_PEFLAG(h);
    uart_port_rx_start(p);
}

uint8_t UART_Transmit(const uint8_t *buf, uint16_t len)
{
    if (!s_uart_ready)
        return 1;

    /* Сначала место проверяется во всех активных портах: сообщение либо
     * встаёт в очередь целиком в оба порта, либо не отправляется вовсе —
     * иначе при заполнении второго порта первый уже получил бы копию и
     * потоки на USART1/USART3 рассинхронизировались бы. */
    for (uint8_t i = 0; i < UART_PORT_COUNT; i++) {
        UartPortCtx *p = &s_ports[i];
        if (p->ready && tx_free(p) < len)
            return 1;
    }

    for (uint8_t i = 0; i < UART_PORT_COUNT; i++) {
        UartPortCtx *p = &s_ports[i];
        if (!p->ready)
            continue;
        for (uint16_t j = 0; j < len; j++) {
            p->tx_ring[p->tx_head] = buf[j];
            p->tx_head = (p->tx_head + 1U) % UART_TX_RING_SIZE;
        }
    }
    return 0;
}

void UART_Task(void)
{
    if (!s_uart_ready)
        return;
    for (uint8_t i = 0; i < UART_PORT_COUNT; i++)
        uart_port_task(&s_ports[i]);
}

uint8_t UART_TxPending(void)
{
    if (!s_uart_ready)
        return 0;

    for (uint8_t i = 0; i < UART_PORT_COUNT; i++) {
        UartPortCtx *p = &s_ports[i];
        if (p->ready && (p->tx_busy || tx_count(p) != 0U))
            return 1U;
    }
    return 0U;
}

uint16_t UART_ReadLine(char *buf, uint16_t size)
{
    if (!s_uart_ready)
        return 0;
    return line_reader_extract(rx_ring, UART_RX_RING_SIZE, rx_head, &rx_tail, buf, size);
}
