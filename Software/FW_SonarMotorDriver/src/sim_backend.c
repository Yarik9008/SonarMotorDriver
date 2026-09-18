/**
 * @file sim_backend.c
 * @brief Виртуальные энкодер и мотор для сборки-имитатора (env:sim).
 *
 * Имитатор — это та же прошивка: main.c, протокол команд, ПИД, профиль
 * движения, скан, синхронизация и телеметрия собираются из общих исходников,
 * без единого условия компиляции внутри логики. Подменяется только слой
 * железа, и делает это сборка (build_src_filter в platformio.ini):
 * - вместо драйвера энкодера (biss_c.c) — виртуальный энкодер, возвращающий
 *   положение модели вала;
 * - вместо фасада TMC2209 (lib/tmc2209) — виртуальный шаговый двигатель,
 *   интегрирующий скомандованную скорость и серии шагов доводки.
 *
 * Поэтому имитатор отвечает по UART ровно как боевая плата (включая ответы,
 * которые раньше приходилось повторять вручную во второй прошивке), но не
 * требует ни энкодера, ни драйвера: чтения всегда валидны (ec:0, m:cl),
 * диагностика проходит, защита от блокировки вала не срабатывает — вал
 * идеально следует за командой.
 *
 * Модель считается раз в тик опроса (1 кГц): BiSS_StartRead() вызывается из
 * обработчика TIM2 каждый тик, там же продвигается положение вала.
 */

#ifndef BUILD_SIM
#error "sim_backend.c собирается только в окружении env:sim (-DBUILD_SIM=1)"
#endif

#include "stm32f1xx_hal.h"
#include "board.h"
#include "biss_c.h"
#include "tmc2209/tmc2209_motor.h"
#include "tmc2209/tmc2209_port_stm32_hal.h"
#include <string.h>

/* Стартовое положение вала: ненулевое, чтобы после сброса был виден выход в
 * «дом» (STARTUP_TARGET_OFFSET_DEG), как на реальной плате. */
#define SIM_START_POS_DEG 37.5

/* --- Состояние модели --- */

static double   s_pos_deg    = SIM_START_POS_DEG; ///< Положение вала, [0,360)
static double   s_rate_deg   = 0.0;   ///< Скорость от set_step_rate, град/тик
static double   s_pend_deg   = 0.0;   ///< Серия шагов доводки, град (на ближайший тик)
static uint8_t  s_enabled    = 0;     ///< ENN: подан ли ток в обмотки

/* Конфигурация «драйвера» — то, что возвращает mcfg */
static uint16_t s_microsteps = TMC2209_MICROSTEPS;
static uint16_t s_run_ma     = TMC2209_IRUN_MA;
static uint16_t s_hold_ma    = TMC2209_IHOLD_MA;

/* Снимок последнего «чтения», готовый к выдаче (аналог результата DMA) */
static BiSS_Reading s_sample;

static double sim_deg_per_step(void)
{
    return 360.0 / ((double)MOTOR_FULL_STEPS_REV * (double)s_microsteps);
}

/* Приведение к кольцу [0,360). Приращение за тик ограничено профилем скорости
 * (не более MAX_SPEED_DEG_S / POLL_FREQ_HZ = 1.2°), поэтому хватает одного
 * вычитания — вызывать fmod с программной арифметикой double в обработчике
 * прерывания 1 кГц не за что. */
static double sim_wrap360(double deg)
{
    while (deg >= 360.0) deg -= 360.0;
    while (deg < 0.0)    deg += 360.0;
    return deg;
}

/**
 * @brief Снимок положения вала в формате чтения энкодера.
 *
 * Положение квантуется в отсчёты энкодера (ENCODER_COUNTS_REV на оборот) —
 * как у настоящего датчика, поэтому телеметрия имитатора имеет ту же
 * дискретность, что и на плате.
 */
static void sim_snapshot(BiSS_Reading *out)
{
    uint32_t counts = (uint32_t)(s_pos_deg / 360.0 * (double)ENCODER_COUNTS_REV);
    if (counts >= ENCODER_COUNTS_REV)
        counts = ENCODER_COUNTS_REV - 1U;   /* защита от округления вверх на границе */

    memset(out, 0, sizeof(*out));
    out->raw_position = counts;
    out->position     = counts;
    out->angle_deg    = (float)((double)counts * 360.0 / (double)ENCODER_COUNTS_REV);
    out->status       = BISS_OK;
}

/**
 * @brief Один тик модели вала.
 *
 * Вал идёт со скомандованной скоростью плюс серия шагов доводки. Без тока в
 * обмотках (ENN = HIGH, команда hold=0 или dis) вал не двигается — так же,
 * как на плате: импульсы STEP уходят в выключенный драйвер впустую.
 */
static void sim_advance_tick(void)
{
    if (s_enabled)
        s_pos_deg = sim_wrap360(s_pos_deg + s_rate_deg + s_pend_deg);
    s_pend_deg = 0.0;
}

/* --- Виртуальный энкодер (интерфейс biss_c.h) --- */

BiSS_Status BiSS_Init(const BiSS_Config *cfg)
{
    (void)cfg;
    s_pos_deg = SIM_START_POS_DEG;
    sim_snapshot(&s_sample);
    return BISS_OK;
}

BiSS_Status BiSS_Read(BiSS_Reading *out)
{
    /* Блокирующее чтение используется только стартовой диагностикой: модель
     * при этом стоит, поэтому серия из ENCODER_DIAG_SAMPLES чтений даёт
     * нулевой разброс и диагностика проходит. */
    sim_snapshot(out);
    return BISS_OK;
}

uint8_t BiSS_StartRead(void)
{
    sim_advance_tick();
    sim_snapshot(&s_sample);
    return 0;   /* 0 = чтение запущено */
}

uint8_t BiSS_IsReady(void)
{
    return 1U;  /* модель отвечает мгновенно */
}

BiSS_Status BiSS_GetResult(BiSS_Reading *out)
{
    *out = s_sample;
    return BISS_OK;
}

void BiSS_Abort(void)
{
    /* Зависшего обмена у модели не бывает */
}

/* --- Виртуальный мотор (интерфейс tmc2209/tmc2209_motor.h) --- */

int tmc2209_motor_init(void)
{
    /* Вывод ENN настраиваем и держим в HIGH: если имитатор окажется прошит в
     * реальную плату, силовая часть TMC2209 останется выключенной. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = ENABLE_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ENABLE_PORT, &gpio);
    HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, GPIO_PIN_SET);
    s_enabled = 0;
    return 0;
}

void tmc2209_motor_task(void)
{
    /* Фоновый обмен с драйвером модели не нужен */
}

int tmc2209_motor_set_enabled(int enabled)
{
    s_enabled = enabled ? 1U : 0U;
    HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN,
                      s_enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
    return 0;
}

void tmc2209_motor_stop(void)
{
    s_rate_deg = 0.0;
    s_pend_deg = 0.0;
}

int tmc2209_motor_is_moving(void)
{
    return (s_rate_deg != 0.0 || s_pend_deg != 0.0) ? 1 : 0;
}

void tmc2209_motor_set_step_rate(uint32_t steps_per_s, int8_t dir_cw)
{
    double v = (double)steps_per_s * sim_deg_per_step() / (double)POLL_FREQ_HZ;
    s_rate_deg = dir_cw ? v : -v;
}

void tmc2209_motor_move_steps(int32_t steps)
{
    /* Серия доводки отрабатывается ближайшим тиком модели: на плате она
     * занимает несколько миллисекунд, и там, и здесь событие «приехали»
     * выставляется следующим тиком (main.c ждёт !tmc2209_motor_is_moving()). */
    s_pend_deg += (double)steps * sim_deg_per_step();
}

void tmc2209_motor_tim4_period_elapsed(void)
{
    /* TIM4 в имитаторе не запускается — обработчик не вызывается */
}

int tmc2209_motor_set_current(uint16_t run_ma, uint16_t hold_ma)
{
    s_run_ma  = run_ma;
    s_hold_ma = hold_ma;
    return 0;
}

int tmc2209_motor_set_microsteps(uint16_t microsteps)
{
    /* Та же проверка, что у настоящего драйвера: степень двойки 1..256 */
    if (microsteps == 0U || microsteps > 256U ||
        (microsteps & (uint16_t)(microsteps - 1U)) != 0U)
        return -2;
    s_microsteps = microsteps;
    return 0;
}

void tmc2209_motor_get_config(tmc2209_motor_config_t *cfg)
{
    cfg->run_ma     = s_run_ma;
    cfg->hold_ma    = s_hold_ma;
    cfg->microsteps = s_microsteps;
    cfg->mode       = TMC2209_MOTOR_CONTROL_STEP_DIR;
    cfg->ready      = 1U;
}

/* --- Заглушки порта: UART драйвера в имитаторе не поднимается --- */

void tmc2209_port_stm32_hal_uart_msp_init(UART_HandleTypeDef *h)
{
    (void)h;
}

void tmc2209_port_stm32_hal_uart_msp_deinit(UART_HandleTypeDef *h)
{
    (void)h;
}
