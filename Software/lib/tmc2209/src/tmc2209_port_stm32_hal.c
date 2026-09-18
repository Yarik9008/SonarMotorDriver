/* tmc2209_port_stm32_hal.c — реализации порта TMC2209 на STM32 HAL. */

#include "tmc2209/tmc2209_port_stm32_hal.h"
#include <stdint.h>
#include <stddef.h>

/* Линия PDN_UART (какой USART и какие выводы) описывается в board.h прошивки.
 * Свой board.h есть не у каждого проекта, поэтому включаем его только если он
 * доступен. */
#if defined(__has_include)
#  if __has_include("board.h")
#    include "board.h"
#  endif
#else
#  include "board.h"
#endif

/* Разводка линии PDN_UART задаётся целиком и без умолчаний.
 *
 * Раньше USART, выводы и тактирование имели независимые умолчания (USART2 на
 * PA2/PA3). Плата, описавшая только USART3 и PB10/PB11 — самый естественный
 * способ развести новую плату, — получала правильные выводы, но тактирование
 * USART2 и GPIOA: обмен молча не шёл, а диагностика показывала лишь «драйвер
 * не отвечает». Теперь недостающий макрос — ошибка сборки, а тактирование
 * выводится из выбранного USART и из фактически заданных портов (см. ниже),
 * так что рассогласовать эти настройки больше нечем. */
#if !defined(TMC2209_UART)
#error "TMC2209: board.h должен задавать TMC2209_UART — USART, к которому подключена линия PDN_UART"
#endif
#if !defined(TMC2209_UART_TX_PORT) || !defined(TMC2209_UART_TX_PIN)
#error "TMC2209: board.h должен задавать TMC2209_UART_TX_PORT и TMC2209_UART_TX_PIN — вывод TX линии PDN_UART"
#endif
#if !defined(TMC2209_UART_RX_PORT) || !defined(TMC2209_UART_RX_PIN)
#error "TMC2209: board.h должен задавать TMC2209_UART_RX_PORT и TMC2209_UART_RX_PIN — вывод RX линии PDN_UART"
#endif
#if defined(TMC2209_UART_CLK_ENABLE) || defined(TMC2209_UART_CLK_DISABLE) || defined(TMC2209_UART_GPIO_CLK_ENABLE)
#error "TMC2209: TMC2209_UART_CLK_ENABLE/DISABLE и TMC2209_UART_GPIO_CLK_ENABLE больше не используются — тактирование выводится из TMC2209_UART и портов TX/RX; уберите их из board.h, иначе настройка уйдёт в пустоту"
#endif

/* ---- Тактирование, выведенное из разводки ---- */

/* Ветка else в цепочках ниже недостижима, пока периферия знакомая. Если
 * TMC2209_UART или порт окажется незнакомым, она не свернётся и сборка
 * упадёт: у GCC — сразу с текстом атрибута error (приём BUILD_BUG_ON из ядра
 * Linux), у остальных компиляторов — на компоновке, потому что тела у функции
 * нет. Тихо остаться без тактирования нельзя ни в том, ни в другом случае. */
#if defined(__GNUC__)
extern void tmc2209_port_unsupported_clk(void)
    __attribute__((error("TMC2209: для выбранного TMC2209_UART или порта GPIO здесь не описано тактирование — добавьте ветку в tmc2209_port_stm32_hal.c")));
#else
extern void tmc2209_port_unsupported_clk(void);
#endif

/* Периферия, которой у конкретного кристалла может не быть: если заголовок МК
 * её не объявляет, соответствующая ветка цепочки просто отсутствует. */
#if defined(USART3)
#  define TMC2209_PORT_USART3_CLK(act)  else if (TMC2209_UART == USART3) { __HAL_RCC_USART3_CLK_##act(); }
#else
#  define TMC2209_PORT_USART3_CLK(act)
#endif
#if defined(UART4)
#  define TMC2209_PORT_UART4_CLK(act)   else if (TMC2209_UART == UART4) { __HAL_RCC_UART4_CLK_##act(); }
#else
#  define TMC2209_PORT_UART4_CLK(act)
#endif
#if defined(UART5)
#  define TMC2209_PORT_UART5_CLK(act)   else if (TMC2209_UART == UART5) { __HAL_RCC_UART5_CLK_##act(); }
#else
#  define TMC2209_PORT_UART5_CLK(act)
#endif
#if defined(GPIOE)
#  define TMC2209_PORT_GPIOE_CLK(port)  else if ((port) == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
#else
#  define TMC2209_PORT_GPIOE_CLK(port)
#endif
#if defined(GPIOF)
#  define TMC2209_PORT_GPIOF_CLK(port)  else if ((port) == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
#else
#  define TMC2209_PORT_GPIOF_CLK(port)
#endif
#if defined(GPIOG)
#  define TMC2209_PORT_GPIOG_CLK(port)  else if ((port) == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }
#else
#  define TMC2209_PORT_GPIOG_CLK(port)
#endif

/* Тактирование выбранного USART; act — ENABLE или DISABLE. Адреса периферии —
 * константы, поэтому сравнения вычисляет компилятор: в код попадает ровно одна
 * запись в RCC, та же, что раньше давал макрос из board.h. */
#define TMC2209_PORT_UART_CLK(act)                                         \
    do {                                                                   \
        if (TMC2209_UART == USART1)      { __HAL_RCC_USART1_CLK_##act(); } \
        else if (TMC2209_UART == USART2) { __HAL_RCC_USART2_CLK_##act(); } \
        TMC2209_PORT_USART3_CLK(act)                                       \
        TMC2209_PORT_UART4_CLK(act)                                        \
        TMC2209_PORT_UART5_CLK(act)                                        \
        else { tmc2209_port_unsupported_clk(); }                           \
    } while (0)

/* Тактирование порта GPIO, на котором стоит вывод линии PDN_UART. */
#define TMC2209_PORT_GPIO_CLK_ENABLE(port)                          \
    do {                                                            \
        if ((port) == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); } \
        else if ((port) == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); } \
        else if ((port) == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); } \
        else if ((port) == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); } \
        TMC2209_PORT_GPIOE_CLK(port)                                \
        TMC2209_PORT_GPIOF_CLK(port)                                \
        TMC2209_PORT_GPIOG_CLK(port)                                \
        else { tmc2209_port_unsupported_clk(); }                    \
    } while (0)

/* ---- Микрозадержка (DWT) ---- */

static void port_delay_us(uint32_t us, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    uint32_t cycles = us * (hal->sysclk_hz / 1000000U);
    /* DWT должен быть инициализирован ранее (например, в board_init) */
    uint32_t t0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - t0) < cycles) { }
}

/* ---- UART ---- */

static int port_uart_tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    if (hal->half_duplex) HAL_HalfDuplex_EnableTransmitter(hal->huart);
    HAL_StatusTypeDef st = HAL_UART_Transmit(hal->huart, (uint8_t *)data, len, timeout_ms);
    if (hal->half_duplex) HAL_HalfDuplex_EnableReceiver(hal->huart);
    return (st == HAL_OK) ? 0 : -1;
}

static int port_uart_rx(uint8_t *data, uint16_t max_len, uint32_t timeout_ms, uint16_t *received, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    *received = 0;
    if (max_len == 0) return 0;

    HAL_StatusTypeDef st = HAL_UART_Receive(hal->huart, &data[0], 1, timeout_ms);
    if (st == HAL_TIMEOUT) return 1;
    if (st != HAL_OK) return -1;
    *received = 1;

    for (uint16_t i = 1; i < max_len; i++) {
        st = HAL_UART_Receive(hal->huart, &data[i], 1, 2);
        if (st != HAL_OK) break;
        (*received)++;
    }
    return (*received == max_len) ? 0 : 1;
}

static void port_uart_rx_flush(void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    UART_HandleTypeDef *h = hal->huart;
    __HAL_UART_CLEAR_OREFLAG(h);
    __HAL_UART_CLEAR_PEFLAG(h);
    __HAL_UART_CLEAR_NEFLAG(h);
    __HAL_UART_CLEAR_FEFLAG(h);
    while (__HAL_UART_GET_FLAG(h, UART_FLAG_RXNE)) (void)h->Instance->DR;
}

/* ---- GPIO и таймер ---- */

static void port_set_enable(uint8_t level, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    HAL_GPIO_WritePin(hal->en_port, hal->en_pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int port_motor_hw_init(void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;

    /* Выводы GPIO */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = hal->step_pin;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(hal->step_port, &gpio);
    HAL_GPIO_WritePin(hal->step_port, hal->step_pin, GPIO_PIN_RESET);

    gpio.Pin   = hal->dir_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(hal->dir_port, &gpio);
    HAL_GPIO_WritePin(hal->dir_port, hal->dir_pin, GPIO_PIN_RESET);

    gpio.Pin   = hal->en_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(hal->en_port, &gpio);
    HAL_GPIO_WritePin(hal->en_port, hal->en_pin, GPIO_PIN_SET); /* Старт с выключенным драйвером */

    /* Инициализация таймера частично выполняется при настройке hal->htim_step;
       здесь при необходимости дополняем конфигурацию ШИМ или полагаемся на готовность. */
    if (HAL_TIM_PWM_Init(hal->htim_step) != HAL_OK) return -1;

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse        = (hal->htim_step->Init.Period + 1) / 2; /* По умолчанию 50% при заданном периоде */
    
    if (HAL_TIM_PWM_ConfigChannel(hal->htim_step, &sConfigOC, hal->tim_channel) != HAL_OK) return -1;

    return 0;
}

static void port_motor_set_dir(int8_t dir_cw, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    HAL_GPIO_WritePin(hal->dir_port, hal->dir_pin, dir_cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void port_motor_set_rate(uint16_t arr, uint16_t ccr, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    __HAL_TIM_SET_AUTORELOAD(hal->htim_step, arr - 1U);
    __HAL_TIM_SET_COMPARE(hal->htim_step, hal->tim_channel, ccr);

    /* Если таймер ещё не запущен — запускаем */
    if (!(hal->htim_step->Instance->CR1 & TIM_CR1_CEN)) {
        /* ARPE=1: на остановленном таймере новые ARR/CCR вступили бы в силу
         * только после первого update — первый период после stop→start шёл бы
         * со старыми регистрами (лишний импульс неверной длительности при
         * каждом старте и смене направления). Форсируем загрузку preload и
         * чистим UIF, чтобы pulse-режим не съел первый шаг. На ходу ARPE не
         * трогаем — там смена периода безглитчевая. */
        hal->htim_step->Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_FLAG(hal->htim_step, TIM_FLAG_UPDATE);
        __HAL_TIM_SET_COUNTER(hal->htim_step, 0);
        HAL_TIM_PWM_Start(hal->htim_step, hal->tim_channel);
    }
}

static void port_motor_stop(void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    HAL_TIM_PWM_Stop(hal->htim_step, hal->tim_channel);
    HAL_GPIO_WritePin(hal->step_port, hal->step_pin, GPIO_PIN_RESET);
}

static uint32_t port_get_tick(void *ctx)
{
    (void)ctx;
    return HAL_GetTick();
}

static void port_debug_print(const char *str, void *ctx)
{
    tmc2209_hal_ctx_t *hal = (tmc2209_hal_ctx_t *)ctx;
    if (hal->debug_fn) hal->debug_fn(str);
}

void tmc2209_port_stm32_hal_uart_msp_init(UART_HandleTypeDef *h)
{
    if (h->Instance != TMC2209_UART) {
        return;
    }

    TMC2209_PORT_UART_CLK(ENABLE);
    TMC2209_PORT_GPIO_CLK_ENABLE(TMC2209_UART_TX_PORT);
    /* Второй порт тактируем, только если RX стоит не на том же порту, что TX;
     * при одинаковых портах сравнение констант ложно и ветки в коде нет. */
    if (TMC2209_UART_RX_PORT != TMC2209_UART_TX_PORT) {
        TMC2209_PORT_GPIO_CLK_ENABLE(TMC2209_UART_RX_PORT);
    }

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = TMC2209_UART_TX_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TMC2209_UART_TX_PORT, &gpio);

    gpio.Pin  = TMC2209_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(TMC2209_UART_RX_PORT, &gpio);
}

void tmc2209_port_stm32_hal_uart_msp_deinit(UART_HandleTypeDef *h)
{
    if (h->Instance != TMC2209_UART) {
        return;
    }

    TMC2209_PORT_UART_CLK(DISABLE);
    HAL_GPIO_DeInit(TMC2209_UART_TX_PORT, TMC2209_UART_TX_PIN);
    HAL_GPIO_DeInit(TMC2209_UART_RX_PORT, TMC2209_UART_RX_PIN);
}

void tmc2209_port_stm32_hal_fill_io(tmc2209_io_t *io, tmc2209_hal_ctx_t *hal)
{
    io->uart_tx       = port_uart_tx;
    io->uart_rx       = port_uart_rx;
    io->uart_rx_flush = port_uart_rx_flush;
    io->delay_us      = port_delay_us;
    io->set_enable    = port_set_enable;
    io->get_tick      = port_get_tick;

    /* Бэкенд мотора (STEP/DIR) подставляем только тогда, когда он настроен:
     * колбэки port_motor_* разыменовывают hal->htim_step и порты STEP/DIR.
     * Плата, которая формирует STEP сама (тестовый стенд FW_Test_TMC2209 ведёт
     * TIM4 из прошивки), оставляет эти поля нулевыми — и первый же вызов, тот
     * же __HAL_TIM_SET_AUTORELOAD(NULL, ...), записал бы по адресу 0x2C, то
     * есть HardFault. Пустые колбэки честнее: фасад мотора проверяет их при
     * инициализации и возвращает понятную ошибку. */
    if (hal->htim_step != NULL && hal->step_port != NULL && hal->dir_port != NULL) {
        io->motor_hw_init  = port_motor_hw_init;
        io->motor_set_dir  = port_motor_set_dir;
        io->motor_set_rate = port_motor_set_rate;
        io->motor_stop     = port_motor_stop;
    } else {
        io->motor_hw_init  = NULL;
        io->motor_set_dir  = NULL;
        io->motor_set_rate = NULL;
        io->motor_stop     = NULL;
    }

    io->debug_print   = hal->debug_fn ? port_debug_print : 0;
    io->ctx           = hal;
}
