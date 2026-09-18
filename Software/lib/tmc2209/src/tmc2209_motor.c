/**
 * @file tmc2209_motor.c
 * @brief Реализация фасада управления мотором.
 *
 * STEP/DIR: непрерывная генерация импульсов по заданной частоте (TIM4 PWM
 * без перезапуска каждый тик). Одноразовые серии — только для доводки.
 *
 * Фасад намертво привязан к плате: он берёт из board.h прошивки выводы
 * STEP/DIR/EN, линию PDN_UART и параметры мотора. Поэтому компилируется он
 * только по явному запросу — когда прошивка задала -DTMC2209_USE_MOTOR_FACADE
 * в build_flags. Без этого флага файл пуст, и прошивке, которая крутит шаг
 * сама (тестовый стенд FW_Test_TMC2209 ведёт TIM4 из своего кода), не нужно
 * объявлять в board.h чужие макросы ради компиляции неиспользуемого кода.
 */

#include "tmc2209/tmc2209_motor.h"

#ifdef TMC2209_USE_MOTOR_FACADE

#include "tmc2209/tmc2209.h"
#include "tmc2209/tmc2209_port_stm32_hal.h"
#include "board.h"
#include <stdint.h>
#include <stddef.h>

/* Параметры, которые фасад берёт из board.h прошивки.
 *
 * У величин, задающих ток, дробление шага, направление, режим работы, адрес
 * на шине и тип линии PDN_UART, умолчаний нет намеренно. Пока библиотека
 * лежала внутри прошивки, опечатка или переименование такого макроса в
 * board.h были ошибкой компиляции. После выноса в общую библиотеку у каждого
 * из них появилось #ifndef-значение «для нового проекта», и та же опечатка
 * уже не роняла сборку, а тихо подставляла чужую величину: на мотор пошло бы
 * 800 мА вместо 600, ток удержания 400 вместо 300, шаг 1/16 вместо 1/256,
 * направление без инверсии — и ни одного диагностического сообщения. Ниже
 * прежнее поведение восстановлено: нет макроса — сборка падает с понятным
 * текстом.
 *
 * Выводы (STEP/DIR/EN, порты UART) и частоты умолчаний не имели и раньше:
 * угадать их нельзя, отсутствие такого макроса — ошибка компиляции. */
#if !defined(MOTOR_DRIVER_MODE_STEP_DIR_VAL) || !defined(MOTOR_DRIVER_MODE_UART_VAL)
#error "TMC2209: board.h должен задавать MOTOR_DRIVER_MODE_STEP_DIR_VAL и MOTOR_DRIVER_MODE_UART_VAL — обозначения режимов драйвера"
#endif
#if !defined(MOTOR_DRIVER_MODE)
#error "TMC2209: board.h должен задавать MOTOR_DRIVER_MODE (MOTOR_DRIVER_MODE_STEP_DIR_VAL — импульсы STEP/DIR от MCU, MOTOR_DRIVER_MODE_UART_VAL — VACTUAL)"
#endif
#if (MOTOR_DRIVER_MODE != MOTOR_DRIVER_MODE_STEP_DIR_VAL) && (MOTOR_DRIVER_MODE != MOTOR_DRIVER_MODE_UART_VAL)
#error "TMC2209: MOTOR_DRIVER_MODE не совпадает ни с MOTOR_DRIVER_MODE_STEP_DIR_VAL, ни с MOTOR_DRIVER_MODE_UART_VAL"
#endif
#if !defined(MOTOR_DIR_INVERT)
#error "TMC2209: board.h должен задавать MOTOR_DIR_INVERT (1 — направление вращения инвертировано разводкой)"
#endif
#if !defined(TMC2209_UART_ADDR)
#error "TMC2209: board.h должен задавать TMC2209_UART_ADDR — адрес драйвера на шине (выводы MS1/MS2)"
#endif
#if !defined(TMC2209_HALF_DUPLEX)
#error "TMC2209: board.h должен задавать TMC2209_HALF_DUPLEX (1 — TX и RX сведены в одну линию PDN_UART)"
#endif
#if !defined(TMC2209_RSENSE_OHM)
#error "TMC2209: board.h должен задавать TMC2209_RSENSE_OHM — номинал токоизмерительного резистора, Ом: из него считаются токи обмоток"
#endif
#if !defined(TMC2209_IRUN_MA)
#error "TMC2209: board.h должен задавать TMC2209_IRUN_MA — ток при движении, мА"
#endif
#if !defined(TMC2209_IHOLD_MA)
#error "TMC2209: board.h должен задавать TMC2209_IHOLD_MA — ток удержания, мА"
#endif
#if !defined(TMC2209_MICROSTEPS)
#error "TMC2209: board.h должен задавать TMC2209_MICROSTEPS — дробление шага (1, 2, 4 ... 256)"
#endif

/* Тайминги обмена — политика связи, а не свойство платы. Умолчания повторяют
 * TMC2209_DEFAULT_CONFIG, совпадают с боевыми значениями и безопасны: ошибка
 * в них не меняет ток в обмотках, а сразу видна по «драйвер не отвечает».
 * Прошивка переопределяет любое из них в board.h. */
#ifndef TMC2209_REPLY_DELAY_US
#define TMC2209_REPLY_DELAY_US  500U    /* Пауза после передачи перед приёмом, мкс */
#endif
#ifndef TMC2209_CFG_SENDDELAY
#define TMC2209_CFG_SENDDELAY   4U      /* SLAVECONF SENDDELAY (0..15) */
#endif

/* А режиму чоппера и задержке снижения тока умолчания не положены: доводом
 * «ошибка сразу видна» их не прикрыть. Потерянный из-за опечатки
 * TMC2209_SPREADCYCLE молча оставит мотор в StealthChop, а TMC2209_TPOWERDOWN
 * так же тихо сменит время удержания тока после остановки; драйвер при этом
 * отвечает как ни в чём не бывало, и «драйвер не отвечает» не появится. Нет
 * диагностики — нет и умолчания: как у боевых параметров выше, отсутствие
 * макроса роняет сборку с понятным текстом. */
#if !defined(TMC2209_TPOWERDOWN)
#error "TMC2209: board.h должен задавать TMC2209_TPOWERDOWN — задержка снижения тока после остановки (0..255)"
#endif
#if !defined(TMC2209_SPREADCYCLE)
#error "TMC2209: board.h должен задавать TMC2209_SPREADCYCLE — режим чоппера (0 — StealthChop, 1 — SpreadCycle)"
#endif

/* ---- Внутреннее состояние ---- */

static tmc2209_t          s_drv;
static UART_HandleTypeDef  s_huart;
static tmc2209_hal_ctx_t   s_hal;
static TIM_HandleTypeDef   s_htim_step;
static uint8_t            s_tmc_ready = 0;

#define DIAG_INTERVAL_MS   500U
static uint32_t            s_diag_last_ms = 0;
static tmc2209_drv_status_t s_diag_cached = {0};
static uint8_t             s_diag_valid   = 0;

static uint16_t s_run_ma     = TMC2209_IRUN_MA;
static uint16_t s_hold_ma    = TMC2209_IHOLD_MA;
static uint16_t s_microsteps = TMC2209_MICROSTEPS;

static uint8_t             g_pwm_running = 0;
static int8_t              g_cur_dir     = -1;
static uint32_t            g_cur_arr     = 0;
static volatile uint32_t   s_pulses_left = 0;

#define TICKS_PER_US     (TIM4_CLK_HZ / 1000000U)
#define PULSE_TICKS      (STEP_PULSE_US * TICKS_PER_US)
#define MIN_PERIOD_TICKS (PULSE_TICKS + 1U)
#define TICKS_PER_POLL   ((TIM4_CLK_HZ / 1000U) * POLL_INTERVAL_MS)

#define STEP_MODE_STOP   0U
#define STEP_MODE_RATE   1U
#define STEP_MODE_PULSE  2U
static uint8_t g_step_mode = STEP_MODE_STOP;

/* ---- Сброс состояния и управление задачей ---- */

/* Прерывание счётчика импульсов существует только в режиме STEP/DIR: в режиме
   UART тело пустое, а все вызовы лежат внутри таких же #if — функция осталась
   бы определённой, но не вызванной (-Wunused-function). Поэтому под #if целиком. */
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
static void step_pulse_irq_enable(uint8_t on)
{
    if (on) {
        __HAL_TIM_ENABLE_IT(&s_htim_step, TIM_IT_UPDATE);
        HAL_NVIC_SetPriority(TIM4_IRQn, IRQ_PRIO_STEP, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
    } else {
        __HAL_TIM_DISABLE_IT(&s_htim_step, TIM_IT_UPDATE);
    }
}
#endif

#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
static void stepdir_stop_internal(void)
{
    s_drv.io.motor_stop(s_drv.io.ctx);
    step_pulse_irq_enable(0);
    s_pulses_left = 0;
    g_pwm_running = 0;
    g_cur_arr     = 0;
    g_step_mode   = STEP_MODE_STOP;
}
#endif

static void tmc2209_motor_reset_runtime_state(void)
{
    g_pwm_running = 0;
    g_cur_dir     = -1;
    g_cur_arr     = 0;
    s_pulses_left = 0;
    g_step_mode   = STEP_MODE_STOP;

    s_run_ma      = TMC2209_IRUN_MA;
    s_hold_ma     = TMC2209_IHOLD_MA;
    s_microsteps  = TMC2209_MICROSTEPS;

    s_diag_last_ms = 0;
    s_diag_cached  = (tmc2209_drv_status_t){0};
    s_diag_valid   = 0;

    s_tmc_ready    = 0;
}

static int map_result(tmc2209_result_t res)
{
    if (res == TMC2209_OK) return 0;
    if (res == TMC2209_ERR_INVALID_ARG) return -2;
    return -3;
}

static int tmc2209_init_backend(void)
{
    s_huart.Instance          = TMC2209_UART;
    s_huart.Init.BaudRate     = TMC2209_UART_BAUDRATE;
    s_huart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_huart.Init.StopBits     = UART_STOPBITS_1;
    s_huart.Init.Parity       = UART_PARITY_NONE;
    s_huart.Init.Mode         = UART_MODE_TX_RX;
    s_huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_huart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&s_huart) != HAL_OK) return -1;

    s_hal.huart       = &s_huart;
    s_hal.en_port     = ENABLE_PORT;
    s_hal.en_pin      = ENABLE_PIN;
    s_hal.sysclk_hz   = SYSCLK_HZ;
    s_hal.half_duplex = TMC2209_HALF_DUPLEX;
    s_hal.debug_fn    = NULL;

    s_htim_step.Instance = TIM4;
    s_htim_step.Init.Prescaler         = 0;
    s_htim_step.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim_step.Init.Period            = 999;
    s_htim_step.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim_step.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    s_hal.htim_step   = &s_htim_step;
    s_hal.tim_channel = TIM_CHANNEL_3;
    s_hal.step_port   = STEP_PORT;
    s_hal.step_pin    = STEP_PIN;
    s_hal.dir_port    = DIR_PORT;
    s_hal.dir_pin     = DIR_PIN;

    tmc2209_io_t io;
    tmc2209_port_stm32_hal_fill_io(&io, &s_hal);

#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    /* Порт подставляет колбэки STEP/DIR только при настроенном бэкенде;
     * их отсутствие — ошибка конфигурации, а не повод падать в HardFault. */
    if (io.motor_hw_init == NULL) return -1;
    if (io.motor_hw_init(&s_hal) != 0) return -1;
#endif

    tmc2209_config_t cfg = TMC2209_DEFAULT_CONFIG;
    cfg.addr           = TMC2209_UART_ADDR;
    cfg.rsense         = TMC2209_RSENSE_OHM;
    cfg.irun_ma        = TMC2209_IRUN_MA;
    cfg.ihold_ma       = TMC2209_IHOLD_MA;
    cfg.microsteps     = TMC2209_MICROSTEPS;
    cfg.reply_delay_us = TMC2209_REPLY_DELAY_US;
    cfg.senddelay      = TMC2209_CFG_SENDDELAY;
    cfg.tpowerdown     = TMC2209_TPOWERDOWN;
    cfg.en_spreadcycle = TMC2209_SPREADCYCLE;

    tmc2209_result_t res = tmc2209_init(&s_drv, &cfg, &io);
    if (res == TMC2209_OK) {
        s_tmc_ready = 1;
        return 0;
    }
    return (int)res;
}

#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
static void stepdir_apply_rate(uint32_t arr, uint32_t ccr, int8_t dir)
{
    if (dir != g_cur_dir) {
        step_pulse_irq_enable(0);
        s_drv.io.motor_stop(s_drv.io.ctx);
        g_pwm_running = 0;
        s_drv.io.motor_set_dir(dir, s_drv.io.ctx);
        g_cur_dir = dir;
    }

    s_drv.io.motor_set_rate((uint16_t)arr, (uint16_t)ccr, s_drv.io.ctx);
    g_pwm_running = 1;
    g_cur_arr     = arr;
}

static void stepdir_rate_internal(uint32_t steps_per_s, int8_t dir)
{
    if (steps_per_s == 0U) {
        stepdir_stop_internal();
        return;
    }

    step_pulse_irq_enable(0);
    s_pulses_left = 0;
    g_step_mode   = STEP_MODE_RATE;

    uint32_t arr = TIM4_CLK_HZ / steps_per_s;
    if (arr < MIN_PERIOD_TICKS) arr = MIN_PERIOD_TICKS;
    if (arr > 65535U)           arr = 65535U;

    uint32_t ccr = (PULSE_TICKS < arr) ? PULSE_TICKS : (arr / 2U);
    stepdir_apply_rate(arr, ccr, dir);
}

static void stepdir_steps_internal(int32_t steps)
{
    if (steps == 0) {
        stepdir_stop_internal();
        return;
    }

    uint32_t n   = (uint32_t)(steps < 0 ? -steps : steps);
    int8_t   dir = (steps > 0) ? 1 : 0;

    step_pulse_irq_enable(0);
    s_drv.io.motor_stop(s_drv.io.ctx);
    g_pwm_running = 0;
    g_step_mode   = STEP_MODE_PULSE;

    if (dir != g_cur_dir) {
        s_drv.io.motor_set_dir(dir, s_drv.io.ctx);
        g_cur_dir = dir;
    }

    uint32_t arr = TICKS_PER_POLL / n;
    if (arr < MIN_PERIOD_TICKS) arr = MIN_PERIOD_TICKS;
    if (arr > 65535U)           arr = 65535U;

    uint32_t ccr = (PULSE_TICKS < arr) ? PULSE_TICKS : (arr / 2U);

    s_pulses_left = n;
    s_drv.io.motor_set_rate((uint16_t)arr, (uint16_t)ccr, s_drv.io.ctx);
    g_pwm_running = 1;
    g_cur_arr     = arr;
    /* Чистим застрявший UIF от предыдущей серии, иначе первое прерывание
     * сработает мгновенно и доводочная серия укоротится на один шаг. */
    __HAL_TIM_CLEAR_FLAG(&s_htim_step, TIM_FLAG_UPDATE);
    step_pulse_irq_enable(1);
}
#endif

/* ---- Инициализация ---- */

int tmc2209_motor_init(void)
{
    tmc2209_motor_reset_runtime_state();
    return tmc2209_init_backend();
}

void tmc2209_motor_task(void)
{
    if (!s_tmc_ready) return;

    uint32_t now = s_drv.io.get_tick(s_drv.io.ctx);
    if (now - s_diag_last_ms < DIAG_INTERVAL_MS)
        return;

    if (tmc2209_motor_is_moving()) {
        s_diag_last_ms = now;
        return;
    }

    tmc2209_drv_status_t st = {0};
    if (tmc2209_get_drv_status(&s_drv, &st) == TMC2209_OK) {
        s_diag_cached = st;
        s_diag_valid  = 1;
    }
    s_diag_last_ms = now;
}

int tmc2209_motor_is_ready(void)
{
    return s_tmc_ready;
}

int tmc2209_motor_set_enabled(int enabled)
{
    if (!s_tmc_ready) return -1;
    tmc2209_result_t res;
    if (enabled) res = tmc2209_enable(&s_drv);
    else         res = tmc2209_disable(&s_drv);

    return map_result(res);
}

/* ---- Управление движением (бэкенд) ---- */

void tmc2209_motor_stop(void)
{
    if (!s_tmc_ready) return;
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    stepdir_stop_internal();
#else
    tmc2209_stop(&s_drv);
#endif
}

int tmc2209_motor_is_moving(void)
{
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    return g_pwm_running || (s_pulses_left > 0U);
#else
    return s_tmc_ready && ((int32_t)s_drv.shadow.vactual != 0);
#endif
}

#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&s_htim_step);
}
#endif

void tmc2209_motor_tim4_period_elapsed(void)
{
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    if (!s_tmc_ready || g_step_mode != STEP_MODE_PULSE)
        return;
    if (s_pulses_left == 0U)
        return;
    s_pulses_left--;
    if (s_pulses_left == 0U)
        stepdir_stop_internal();
#else
    (void)0;
#endif
}

void tmc2209_motor_set_step_rate(uint32_t steps_per_s, int8_t dir_cw)
{
    if (!s_tmc_ready) return;
#if MOTOR_DIR_INVERT
    dir_cw = dir_cw ? 0 : 1;
#endif

#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    stepdir_rate_internal(steps_per_s, dir_cw);
#else
    if (steps_per_s == 0U) {
        tmc2209_stop(&s_drv);
        return;
    }
    int32_t v = (int32_t)steps_per_s * 256;
    if (dir_cw == 0)
        v = -v;
    if (v > 8388607)  v = 8388607;
    if (v < -8388608) v = -8388608;
    tmc2209_set_vactual(&s_drv, v);
#endif
}

void tmc2209_motor_move_steps(int32_t steps)
{
    if (!s_tmc_ready) return;
#if MOTOR_DIR_INVERT
    steps = -steps;
#endif

#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_STEP_DIR_VAL
    stepdir_steps_internal(steps);
#else
    if (steps == 0) {
        tmc2209_stop(&s_drv);
        return;
    }
    int32_t v = steps * 256;
    if (v > 8388607)  v = 8388607;
    if (v < -8388608) v = -8388608;
    tmc2209_set_vactual(&s_drv, v);
#endif
}

void tmc2209_motor_move_velocity(int32_t velocity)
{
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_UART_VAL
    if (!s_tmc_ready) return;
    if (velocity > 8388607)  velocity = 8388607;
    if (velocity < -8388608) velocity = -8388608;
    tmc2209_set_vactual(&s_drv, velocity);
#else
    (void)velocity;
#endif
}

int tmc2209_motor_set_current(uint16_t run_ma, uint16_t hold_ma)
{
    if (!s_tmc_ready) return -1;
    tmc2209_result_t res = tmc2209_set_current(&s_drv, run_ma, hold_ma);
    if (res == TMC2209_OK) {
        s_run_ma  = run_ma;
        s_hold_ma = hold_ma;
    }
    return map_result(res);
}

int tmc2209_motor_set_microsteps(uint16_t microsteps)
{
    if (!s_tmc_ready) return -1;
    tmc2209_result_t res = tmc2209_set_microsteps(&s_drv, microsteps);
    if (res == TMC2209_OK) {
        s_microsteps = microsteps;
    }
    return map_result(res);
}

void tmc2209_motor_get_config(tmc2209_motor_config_t *cfg)
{
    if (!cfg) return;
    cfg->run_ma     = s_run_ma;
    cfg->hold_ma    = s_hold_ma;
    cfg->microsteps = s_microsteps;
    cfg->mode       = tmc2209_motor_get_control_mode();
    cfg->ready      = s_tmc_ready;
}

tmc2209_motor_mode_t tmc2209_motor_get_control_mode(void)
{
#if MOTOR_DRIVER_MODE == MOTOR_DRIVER_MODE_UART_VAL
    return TMC2209_MOTOR_CONTROL_UART;
#else
    return TMC2209_MOTOR_CONTROL_STEP_DIR;
#endif
}

int tmc2209_motor_get_version(uint8_t *version)
{
    if (!s_tmc_ready || !version) return -1;
    return (tmc2209_get_version(&s_drv, version) == TMC2209_OK) ? 0 : -1;
}

int tmc2209_motor_get_drv_status(tmc2209_drv_status_t *st)
{
    if (!s_tmc_ready || !st) return -1;
    return (tmc2209_get_drv_status(&s_drv, st) == TMC2209_OK) ? 0 : -1;
}

int tmc2209_motor_get_cached_drv_status(tmc2209_drv_status_t *st)
{
    if (!s_diag_valid || !st) return -1;
    *st = s_diag_cached;
    return 0;
}

#endif /* TMC2209_USE_MOTOR_FACADE */
