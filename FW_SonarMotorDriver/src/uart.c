/**
 * @file uart.c
 * @brief Реализация командного UART через DMA.
 *
 * Особенности реализации:
 * - Прием (RX): DMA в режиме Circular Buffer + прерывание по IDLE-линии. Данные
 *   переносятся из DMA-буфера в программный кольцевой буфер при каждом событии IDLE
 *   или заполнении половины/всего DMA-буфера. Это позволяет эффективно принимать
 *   пакеты произвольной длины без прерываний на каждый байт.
 * - Передача (TX): Программный кольцевой буфер + DMA в режиме Normal. Функция
 *   UART_Task() автоматически запускает передачу следующего непрерывного куска
 *   данных из буфера, как только DMA освобождается.
 * - Ошибки: При сбое инициализации DMA или периферии состояние "сбрасывается"
 *   в безопасное (s_uart_ready = 0).
 */

#include "uart.h"
#include "board.h"
#include "line_reader.h"

/* ---- Дескрипторы DMA ---- */

static DMA_HandleTypeDef hdma_uart_rx;
static DMA_HandleTypeDef hdma_uart_tx;

/* ---- Дескриптор UART ---- */

static UART_HandleTypeDef huart;

/* ---- Приём: циклический буфер DMA + программный кольцевой буфер ---- */

/* 
 * Кольцевой буфер приема.
 * dma_rx_buf — "сырой" буфер, куда пишет DMA.
 * rx_ring — буфер, куда мы копируем данные для разбора строковым ридером.
 * dma_rx_prev — хранит индекс байта, который мы обработали последним в dma_rx_buf.
 */
static uint8_t           dma_rx_buf[UART_RX_DMA_SIZE];
static volatile uint16_t dma_rx_prev = 0;       /* последний обработанный индекс в dma_rx_buf */

static uint8_t           rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static uint16_t          rx_tail = 0;

/* Переписывает новые байты из DMA-буфера в rx_ring.
 * Вызывается из IDLE IRQ и DMA HT/TC IRQ.
 * Безопасно вызывать из нескольких источников — всегда проверяем текущую
 * позицию NDTR и сравниваем с предыдущей.
 */
static void uart_rx_drain(void)
{
    /* Позиция, до которой DMA уже записал данные:
     * write_pos = размер - оставшиеся (NDTR). Circular DMA может обернуться. */
    uint16_t write_pos = UART_RX_DMA_SIZE -
                         (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_uart_rx);

    uint16_t prev = dma_rx_prev;
    if (write_pos == prev)
        return;

    /* Обрабатываем новые байты (учитываем circular wrap-around) */
    while (prev != write_pos) {
        uint8_t c = dma_rx_buf[prev];
        prev = (prev + 1U) % UART_RX_DMA_SIZE;

        uint16_t next = (rx_head + 1U) % UART_RX_RING_SIZE;
        if (next != rx_tail) {          /* не переполнено */
            rx_ring[rx_head] = c;
            rx_head = next;
        }
        /* else — тихое переполнение: байт теряется, но буфер не заклинивает */
    }

    dma_rx_prev = write_pos;
}

/* 
 * Кольцевой буфер передачи.
 * tx_busy — флаг того, что DMA в данный момент занят передачей блока.
 * s_uart_ready — флаг успешной инициализации всего модуля.
 */
static uint8_t           tx_ring[UART_TX_RING_SIZE];
static volatile uint16_t tx_head    = 0;
static volatile uint16_t tx_tail    = 0;
static volatile uint8_t  tx_busy    = 0;   /* 1 — DMA передача активна */
static volatile uint8_t  s_uart_ready = 0;
static volatile uint8_t  s_msp_dma_error = 0;

static uint16_t tx_count(void)
{
    uint16_t h = tx_head, t = tx_tail;
    return (h >= t) ? (h - t) : (UART_TX_RING_SIZE - t + h);
}

static uint16_t tx_free(void)
{
    return (UART_TX_RING_SIZE - 1U) - tx_count();
}

static void uart_reset_runtime_state(void)
{
    dma_rx_prev = 0;
    rx_head = 0;
    rx_tail = 0;
    tx_head = 0;
    tx_tail = 0;
    tx_busy = 0;
}

static void uart_rollback_init(void)
{
    HAL_UART_DeInit(&huart);
    uart_reset_runtime_state();
    s_uart_ready = 0;
    s_msp_dma_error = 0;
}

/* ---- Инициализация ---- */

int UART_Init(void)
{
    uart_reset_runtime_state();
    s_uart_ready = 0;
    s_msp_dma_error = 0;

    huart.Instance          = UART_INSTANCE;
    huart.Init.BaudRate     = UART_BAUDRATE;
    huart.Init.WordLength   = UART_WORDLENGTH_8B;
    huart.Init.StopBits     = UART_STOPBITS_1;
    huart.Init.Parity       = UART_PARITY_NONE;
    huart.Init.Mode         = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart) != HAL_OK || s_msp_dma_error) {
        uart_rollback_init();
        return -1;
    }

    /* Запустить DMA RX в circular режиме */
    if (HAL_UART_Receive_DMA(&huart, dma_rx_buf, UART_RX_DMA_SIZE) != HAL_OK) {
        uart_rollback_init();
        return -1;
    }

    /* Включить IDLE interrupt только если DMA успешно запущен */
    __HAL_UART_ENABLE_IT(&huart, UART_IT_IDLE);
    s_uart_ready = 1;
    
    return 0;
}

/* ---- Инициализация HAL MSP ---- */

void UART_CommandMspInit(UART_HandleTypeDef *h)
{
    if (h->Instance != UART_INSTANCE) {
        return;
    }

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* GPIO */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = UART_TX_PIN; gpio.Mode = GPIO_MODE_AF_PP; gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UART_TX_PORT, &gpio);
    gpio.Pin = UART_RX_PIN; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(UART_RX_PORT, &gpio);

    /* DMA RX: DMA1_Channel5 — USART1_RX, circular */
    hdma_uart_rx.Instance                 = DMA1_Channel5;
    hdma_uart_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_uart_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_uart_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_uart_rx) != HAL_OK) {
        s_msp_dma_error = 1;
        return;
    }
    __HAL_LINKDMA(h, hdmarx, hdma_uart_rx);

    /* DMA TX: DMA1_Channel4 — USART1_TX, normal */
    hdma_uart_tx.Instance                 = DMA1_Channel4;
    hdma_uart_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_uart_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart_tx.Init.Mode                = DMA_NORMAL;
    hdma_uart_tx.Init.Priority            = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart_tx) != HAL_OK) {
        HAL_DMA_DeInit(&hdma_uart_rx);
        h->hdmarx = NULL;
        s_msp_dma_error = 1;
        return;
    }
    __HAL_LINKDMA(h, hdmatx, hdma_uart_tx);

    /* DMA IRQs */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, IRQ_PRIO_UART, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, IRQ_PRIO_UART, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    /* USART1 IRQ (нужен для IDLE line detection) */
    HAL_NVIC_SetPriority(USART1_IRQn, IRQ_PRIO_UART, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void UART_CommandMspDeInit(UART_HandleTypeDef *h)
{
    if (h->Instance != UART_INSTANCE) {
        return;
    }

    HAL_DMA_DeInit(&hdma_uart_rx);
    HAL_DMA_DeInit(&hdma_uart_tx);
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(UART_TX_PORT, UART_TX_PIN);
    HAL_GPIO_DeInit(UART_RX_PORT, UART_RX_PIN);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
}

/* ---- Обработчики прерываний ---- */

/* Прерывание USART1: обработка IDLE и завершения DMA от UART */
void USART1_IRQHandler(void)
{
    /* Обнаружена линия IDLE — сброс флага и перенос новых байт в кольцевой буфер */
    if (__HAL_UART_GET_FLAG(&huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart);
        uart_rx_drain();
    }
    HAL_UART_IRQHandler(&huart);
}

/* DMA1_Channel4 — завершение передачи USART1_TX */
void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart_tx);
}

/* DMA1_Channel5 — DMA приёма USART1_RX (полупередача/полная передача) */
void DMA1_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart_rx);
    uart_rx_drain();
}

/* ---- Колбэки HAL DMA ---- */

/* Завершение передачи по DMA — сдвиг tail и запуск следующего блока */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance != UART_INSTANCE) return;
    tx_busy = 0;
    /* Следующая передача будет запущена в UART_Task() при следующем вызове */
}

/* ---- Публичный API ---- */

uint8_t UART_Transmit(const uint8_t *buf, uint16_t len)
{
    if (!s_uart_ready) return 1;
    if (tx_free() < len) return 1;      /* Нет места — данные отброшены */
    for (uint16_t i = 0; i < len; i++) {
        tx_ring[tx_head] = buf[i];
        tx_head = (tx_head + 1U) % UART_TX_RING_SIZE;
    }
    return 0;
}

/* UART_Task() вызывается из main loop.
 * Запускает DMA TX, если DMA свободен и в ring есть данные.
 * Не блокирует. CPU не тратится на отправку байтов.
 */
void UART_Task(void)
{
    if (!s_uart_ready) return;
    if (tx_busy) return;
    uint16_t cnt = tx_count();
    if (!cnt) return;

    /* Непрерывный блок от tail до конца буфера или до head */
    uint16_t chunk = (tx_tail + cnt > UART_TX_RING_SIZE)
                     ? (UART_TX_RING_SIZE - tx_tail) : cnt;

    tx_busy = 1;
    /* Продвигаем tail сразу — DMA будет читать из tx_ring[tx_tail..tail+chunk-1] */
    uint16_t old_tail = tx_tail;
    tx_tail = (tx_tail + chunk) % UART_TX_RING_SIZE;

    if (HAL_UART_Transmit_DMA(&huart, &tx_ring[old_tail], chunk) != HAL_OK) {
        /* Если DMA не запустился — откатываемся */
        tx_tail = old_tail;
        tx_busy = 0;
    }
}

uint8_t UART_TxPending(void)
{
    if (!s_uart_ready) return 0;
    return (tx_busy || (tx_count() != 0U)) ? 1U : 0U;
}

uint16_t UART_ReadLine(char *buf, uint16_t size)
{
    if (!s_uart_ready) return 0;
    return line_reader_extract(rx_ring, UART_RX_RING_SIZE, rx_head, &rx_tail, buf, size);
}
