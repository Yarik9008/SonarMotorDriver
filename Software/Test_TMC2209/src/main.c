/* Test_TMC2209 — пример UART CLI с библиотекой TMC2209 (STM32F103C8).
 *
 * Перенесено с эталонной реализации на STM32F446RE + USB CDC.
 * CLI по USART1 (PA9=TX, PA10=RX), обмен с TMC2209 по USART2 (PA2=TX, PA3=RX).
 *
 * ВНИМАНИЕ: линию питания драйвера VS необходимо ВСЕГДА подключать через
 * дополнительный электролитический конденсатор (100–470 мкФ)!
 *
 * Команды: i m<N> ms<N> ir<N> ih<N> s e d st t v a r diag c p u h
 */

#include "board.h"
#include "tmc2209/tmc2209.h"
#include "tmc2209/tmc2209_port_stm32_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define CMD_BUF_SIZE   64
#define RSP_BUF_SIZE   160
#define CS_INTERVAL_MS 100U

/* ---- Режим приложения: управление по UART (VACTUAL) или по STEP/DIR ---- */

typedef enum { APP_MODE_UART = 0, APP_MODE_STEP_DIR } app_mode_t;

/* ---- Состояние приложения ---- */

static tmc2209_t          s_drv;
static tmc2209_hal_ctx_t  s_hal_ctx;
static UART_HandleTypeDef s_huart_tmc;
static UART_HandleTypeDef s_huart_cli;
static TIM_HandleTypeDef  s_htim_step;

static app_mode_t s_mode          = APP_MODE_UART;
static uint8_t    s_tim_running   = 0;
static uint8_t    s_continuous_cs = 0;
static uint8_t    s_drv_ready     = 0;

static char cmd[CMD_BUF_SIZE];
static char rsp[RSP_BUF_SIZE];

/* ---- Опережающие объявления ---- */

static void SystemClock_Config(void);
static void gpio_hw_init(void);
static void uart_hw_init(void);
static void step_tim_init(void);
static void step_pwm_start(uint32_t steps_per_sec);
static void step_pwm_stop(void);
static void app_init_driver(void);

/* ---- Вспомогательные функции CLI (USART1) ---- */

static void tx(const char *s)
{
    HAL_UART_Transmit(&s_huart_cli, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

static void debug_to_cli(const char *s) { tx(s); }

/* ==== STEP/DIR: генерация ШИМ на уровне приложения (не входит в библиотеку TMC2209) ==== */

#define STEP_CNT_HZ (STEP_TIM_CLK_HZ / (STEP_TIM_PSC + 1U))

static void step_tim_init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    s_htim_step.Instance               = STEP_TIM;
    s_htim_step.Init.Prescaler         = STEP_TIM_PSC;
    s_htim_step.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim_step.Init.Period            = 0xFFFF;
    s_htim_step.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim_step.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&s_htim_step);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = STEP_TIM_PULSE_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&s_htim_step, &oc, STEP_TIM_CH);
    s_tim_running = 0;
}

static void step_pin_as_gpio(void)
{
    if (s_tim_running) {
        HAL_TIM_PWM_Stop(&s_htim_step, STEP_TIM_CH);
        s_tim_running = 0;
    }
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = STEP_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STEP_PORT, &gpio);
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
}

static void step_pin_as_pwm(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = STEP_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(STEP_PORT, &gpio);
}

static void step_pwm_start(uint32_t steps_per_sec)
{
    if (steps_per_sec == 0) {
        if (s_tim_running) { HAL_TIM_PWM_Stop(&s_htim_step, STEP_TIM_CH); s_tim_running = 0; }
        return;
    }
    uint32_t arr = STEP_CNT_HZ / steps_per_sec;
    if (arr < 4)      arr = 4;
    if (arr > 0xFFFF) arr = 0xFFFF;
    __HAL_TIM_SET_AUTORELOAD(&s_htim_step, arr - 1);
    __HAL_TIM_SET_COMPARE(&s_htim_step, STEP_TIM_CH, STEP_TIM_PULSE_US);
    if (!s_tim_running) {
        step_pin_as_pwm();
        HAL_TIM_PWM_Start(&s_htim_step, STEP_TIM_CH);
        s_tim_running = 1;
    }
}

static void step_pwm_stop(void)
{
    if (s_tim_running) { HAL_TIM_PWM_Stop(&s_htim_step, STEP_TIM_CH); s_tim_running = 0; }
    step_pin_as_gpio();
}

/* ==== Движение и останов на уровне приложения (через UART или STEP/DIR) ==== */

static void app_move(int32_t value)
{
    if (s_mode == APP_MODE_UART) {
        tmc2209_set_vactual(&s_drv, value);
    } else {
        if (value == 0) { step_pwm_stop(); return; }
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN,
                          (value > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        step_pwm_start((value > 0) ? (uint32_t)value : (uint32_t)(-value));
    }
}

static void app_stop(void)
{
    if (s_mode == APP_MODE_UART)
        tmc2209_stop(&s_drv);
    else
        step_pwm_stop();
}

static void app_set_mode(app_mode_t mode)
{
    if (mode == s_mode) return;
    if (mode == APP_MODE_STEP_DIR) {
        tmc2209_set_vactual(&s_drv, 0);
        s_mode = APP_MODE_STEP_DIR;
    } else {
        step_pwm_stop();
        s_mode = APP_MODE_UART;
    }
}

/* ==== Вывод телеметрии ==== */

static void output_telemetry(void)
{
    uint32_t v;

    tmc2209_gstat_t gs;
    if (tmc2209_get_gstat(&s_drv, &gs) == TMC2209_OK) {
        snprintf(rsp, sizeof(rsp),
                 "GSTAT=0x%02X reset=%u drv_err=%u uv_cp=%u\r\n",
                 gs.reset | (gs.drv_err << 1) | (gs.uv_cp << 2),
                 gs.reset, gs.drv_err, gs.uv_cp);
        tx(rsp);
    } else tx("GSTAT=?\r\n");

    uint8_t ifcnt;
    if (tmc2209_get_ifcnt(&s_drv, &ifcnt) == TMC2209_OK) {
        snprintf(rsp, sizeof(rsp), "IFCNT=%u\r\n", ifcnt);
        tx(rsp);
    } else tx("IFCNT=?\r\n");

    tmc2209_ioin_t ioin;
    if (tmc2209_get_ioin(&s_drv, &ioin) == TMC2209_OK) {
        snprintf(rsp, sizeof(rsp),
                 "IOIN ENN=%u MS1=%u MS2=%u DIAG=%u STEP=%u DIR=%u VERSION=0x%02X\r\n",
                 ioin.enn, ioin.ms1, ioin.ms2, ioin.diag,
                 ioin.step, ioin.dir, ioin.version);
        tx(rsp);
    } else tx("IOIN=?\r\n");

    uint32_t tstep;
    if (tmc2209_get_tstep(&s_drv, &tstep) == TMC2209_OK) {
        snprintf(rsp, sizeof(rsp), "TSTEP=%lu\r\n", (unsigned long)tstep);
        tx(rsp);
    } else tx("TSTEP=?\r\n");

    uint16_t sg;
    if (tmc2209_get_sg_result(&s_drv, &sg) == TMC2209_OK) {
        snprintf(rsp, sizeof(rsp), "SG_RESULT=%u\r\n", sg);
        tx(rsp);
    } else tx("SG_RESULT=?\r\n");

    if (tmc2209_read_reg(&s_drv, TMC2209_REG_MSCNT, &v) == TMC2209_OK) {
        snprintf(rsp, sizeof(rsp), "MSCNT=%lu\r\n", (unsigned long)(v & 0x3FF));
        tx(rsp);
    } else tx("MSCNT=?\r\n");

    if (tmc2209_read_reg(&s_drv, TMC2209_REG_MSCURACT, &v) == TMC2209_OK) {
        int32_t cur_a = (int32_t)(v & 0x1FF);
        if (cur_a & 0x100) cur_a -= 512;
        int32_t cur_b = (int32_t)((v >> 16) & 0x1FF);
        if (cur_b & 0x100) cur_b -= 512;
        snprintf(rsp, sizeof(rsp), "MSCURACT CUR_A=%ld CUR_B=%ld\r\n",
                 (long)cur_a, (long)cur_b);
        tx(rsp);
    } else tx("MSCURACT=?\r\n");

    if (tmc2209_read_reg(&s_drv, TMC2209_REG_CHOPCONF, &v) == TMC2209_OK) {
        snprintf(rsp, sizeof(rsp), "CHOPCONF=0x%08lX MRES=%lu\r\n",
                 (unsigned long)v, (unsigned long)((v >> 24) & 0x0F));
        tx(rsp);
    } else tx("CHOPCONF=?\r\n");

    if (tmc2209_read_reg(&s_drv, TMC2209_REG_DRV_STATUS, &v) == TMC2209_OK) {
        snprintf(rsp, sizeof(rsp),
                 "DRV_STATUS=0x%08lX stst=%lu stealth=%lu cs_actual=%lu "
                 "s2ga=%lu s2gb=%lu ola=%lu olb=%lu ot=%lu otpw=%lu\r\n",
                 (unsigned long)v,
                 (v >> 31) & 1, (v >> 30) & 1, (v >> 16) & 0x1F,
                 (v >> 2)  & 1, (v >> 3)  & 1, (v >> 6)  & 1,
                 (v >> 7)  & 1, (v >> 1)  & 1, (v >> 0)  & 1);
        tx(rsp);
    } else tx("DRV_STATUS=?\r\n");

    if (tmc2209_read_reg(&s_drv, TMC2209_REG_PWM_SCALE, &v) == TMC2209_OK) {
        int32_t auto_off = (int32_t)((v >> 16) & 0x1FF);
        if (auto_off & 0x100) auto_off -= 512;
        snprintf(rsp, sizeof(rsp), "PWM_SCALE sum=%lu auto=%ld\r\n",
                 (unsigned long)(v & 0xFF), (long)auto_off);
        tx(rsp);
    } else tx("PWM_SCALE=?\r\n");
}

/* ==== Обработчик команд CLI (USART1, опрос побайтово) ==== */

static void process_cmd(void)
{
    uint8_t ch;
    static uint16_t ptr = 0;

    if (HAL_UART_Receive(&s_huart_cli, &ch, 1, 0) != HAL_OK) return;

    if (ch == '\r' || ch == '\n') {
        if (ptr == 0) return;
        cmd[ptr] = '\0';
        ptr = 0;
    } else {
        if (ptr < CMD_BUF_SIZE - 1) cmd[ptr++] = ch;
        return;
    }

    const char *mode_str = (s_mode == APP_MODE_UART) ? "UA" : "SD";
    uint16_t n = (uint16_t)strlen(cmd);

    if (cmd[0] == 'h' && n == 1) {
        tx("i  init\r\n"
           "m<N> move (UART:VACTUAL / SD:steps/s)\r\n"
           "ms<N> microsteps (1,2,4,8,16,32,64,128,256)\r\n"
           "ir<N> run current, mA\r\n"
           "ih<N> hold current, mA\r\n"
           "s  stop\r\n"
           "e  enable\r\n"
           "d  disable\r\n"
           "p  STEP/DIR mode\r\n"
           "u  UART mode\r\n"
           "st status\r\n"
           "t  telemetry (all regs)\r\n"
           "v  version\r\n"
           "a  scan addrs 0..3\r\n"
           "r  read IOIN (single)\r\n"
           "diag  transport diag\r\n"
           "c  current x10 (cs loop, s=stop)\r\n"
           "h  help\r\n");
    }
    else if (cmd[0] == 'i' && n == 1) {
        tx("init\r\n");
        app_init_driver();
    }
    else if (n >= 3 && cmd[0] == 'i' && cmd[1] == 'r') {
        uint16_t ma = (uint16_t)atoi(cmd + 2);
        if (tmc2209_set_run_current(&s_drv, ma) == TMC2209_OK) {
            snprintf(rsp, sizeof(rsp), "irun=%u mA\r\n", ma);
            tx(rsp);
        } else {
            tx("ir: error (check rsense)\r\n");
        }
    }
    else if (n >= 3 && cmd[0] == 'i' && cmd[1] == 'h') {
        uint16_t ma = (uint16_t)atoi(cmd + 2);
        if (tmc2209_set_hold_current(&s_drv, ma) == TMC2209_OK) {
            snprintf(rsp, sizeof(rsp), "ihold=%u mA\r\n", ma);
            tx(rsp);
        } else {
            tx("ih: error (must be <= irun)\r\n");
        }
    }
    else if (n >= 3 && cmd[0] == 'm' && cmd[1] == 's') {
        uint16_t ms = (uint16_t)atoi(cmd + 2);
        if (tmc2209_set_microsteps(&s_drv, ms) == TMC2209_OK) {
            snprintf(rsp, sizeof(rsp), "microsteps=%u\r\n", ms);
            tx(rsp);
        } else {
            tx("ms: invalid (use 1,2,4,8,16,32,64,128,256)\r\n");
        }
    }
    else if (cmd[0] == 'm') {
        int32_t val = (int32_t)atoi(cmd + 1);
        app_move(val);
        snprintf(rsp, sizeof(rsp), "%s=%ld\r\n", mode_str, (long)val);
        tx(rsp);
    }
    else if (cmd[0] == 's' && n == 1) {
        s_continuous_cs = 0;
        app_stop();
        tx("stop\r\n");
    }
    else if (cmd[0] == 's' && cmd[1] == 't') {
        uint32_t st = 0;
        if (tmc2209_read_reg(&s_drv, TMC2209_REG_DRV_STATUS, &st) == TMC2209_OK)
            snprintf(rsp, sizeof(rsp), "DRV=0x%08lX [%s]\r\n",
                     (unsigned long)st, mode_str);
        else
            snprintf(rsp, sizeof(rsp), "DRV= transport error [%s]\r\n", mode_str);
        tx(rsp);
    }
    else if (cmd[0] == 'e' && n == 1) {
        tmc2209_enable(&s_drv);
        tx("on\r\n");
    }
    else if (cmd[0] == 'd' && n >= 1 && cmd[1] != 'i') {
        tmc2209_disable(&s_drv);
        tx("off\r\n");
    }
    else if (n >= 4 && memcmp(cmd, "diag", 4) == 0) {
        snprintf(rsp, sizeof(rsp),
                 "addr=%u rsense=%.2f irun=%u ihold=%u mstep=%u delay=%uus\r\n",
                 s_drv.cfg.addr, (double)s_drv.cfg.rsense,
                 s_drv.cfg.irun_ma, s_drv.cfg.ihold_ma,
                 s_drv.cfg.microsteps, s_drv.cfg.reply_delay_us);
        tx(rsp);
        tx("Reading IOIN...\r\n");
        tmc2209_ioin_t ioin;
        if (tmc2209_get_ioin(&s_drv, &ioin) == TMC2209_OK) {
            snprintf(rsp, sizeof(rsp), "IOIN VERSION=0x%02X\r\n", ioin.version);
            tx(rsp);
        } else {
            tx("IOIN read FAILED\r\n");
        }
    }
    else if (cmd[0] == 'p' && n == 1) {
        app_set_mode(APP_MODE_STEP_DIR);
        tx("mode:SD\r\n");
    }
    else if (cmd[0] == 'u' && n == 1) {
        app_set_mode(APP_MODE_UART);
        tx("mode:UA\r\n");
    }
    else if (cmd[0] == 't' && n == 1) {
        output_telemetry();
    }
    else if (cmd[0] == 'a' && n == 1) {
        tx("scan addrs 0..3:\r\n");
        for (uint8_t a = 0; a <= 3; a++) {
            uint32_t v = 0;
            if (tmc2209_read_reg_addr(&s_drv, a, TMC2209_REG_IOIN, &v) == TMC2209_OK
                && ((v >> 24) & 0xFF) == TMC2209_VERSION_EXPECTED) {
                snprintf(rsp, sizeof(rsp), "  addr %u: TMC2209 OK (IOIN=0x%08lX)\r\n",
                         a, (unsigned long)v);
                tx(rsp);
            }
        }
    }
    else if (cmd[0] == 'r' && n == 1) {
        tmc2209_ioin_t ioin;
        if (tmc2209_get_ioin(&s_drv, &ioin) == TMC2209_OK) {
            snprintf(rsp, sizeof(rsp),
                     "IOIN VERSION=0x%02X ENN=%u MS1=%u MS2=%u\r\n",
                     ioin.version, ioin.enn, ioin.ms1, ioin.ms2);
            tx(rsp);
        } else {
            tx("IOIN read FAILED\r\n");
        }
    }
    else if (cmd[0] == 'v' && n == 1) {
        uint8_t ver = 0;
        if (tmc2209_get_version(&s_drv, &ver) == TMC2209_OK) {
            snprintf(rsp, sizeof(rsp), "IC=0x%02X%s\r\n",
                     ver, (ver == 0x21) ? " OK" : " ??");
            tx(rsp);
        } else {
            tx("IC= transport error (IOIN read failed)\r\n");
        }
    }
    else if (cmd[0] == 'c' && n == 1) {
        s_continuous_cs = 1;
        tx("cs loop (s=stop)\r\n");
    }
    else {
        tx("?\r\n");
    }
}

/* ==== HAL UART MspInit: настройка GPIO под оба UART на плате ==== */

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    GPIO_InitTypeDef gpio = {0};
    if (h->Instance == TMC2209_UART) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin   = TMC2209_UART_TX_PIN;
        gpio.Mode  = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(TMC2209_UART_TX_PORT, &gpio);
        gpio.Pin   = TMC2209_UART_RX_PIN;
        gpio.Mode  = GPIO_MODE_INPUT;
        gpio.Pull  = GPIO_PULLUP;
        HAL_GPIO_Init(TMC2209_UART_RX_PORT, &gpio);
    }
    else if (h->Instance == CLI_UART) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin   = CLI_UART_TX_PIN;
        gpio.Mode  = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(CLI_UART_TX_PORT, &gpio);
        gpio.Pin   = CLI_UART_RX_PIN;
        gpio.Mode  = GPIO_MODE_INPUT;
        gpio.Pull  = GPIO_PULLUP;
        HAL_GPIO_Init(CLI_UART_RX_PORT, &gpio);
    }
}

/* ==== Инициализация аппаратуры ==== */

static void gpio_hw_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, GPIO_PIN_SET);
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = ENABLE_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ENABLE_PORT, &gpio);
    gpio.Pin = DIR_PIN;
    HAL_GPIO_Init(DIR_PORT, &gpio);
    gpio.Pin = STEP_PIN;
    HAL_GPIO_Init(STEP_PORT, &gpio);
}

static void uart_hw_init(void)
{
    s_huart_tmc.Instance          = TMC2209_UART;
    s_huart_tmc.Init.BaudRate     = TMC2209_UART_BAUDRATE;
    s_huart_tmc.Init.WordLength   = UART_WORDLENGTH_8B;
    s_huart_tmc.Init.StopBits     = UART_STOPBITS_1;
    s_huart_tmc.Init.Parity       = UART_PARITY_NONE;
    s_huart_tmc.Init.Mode         = UART_MODE_TX_RX;
    s_huart_tmc.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_huart_tmc.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_huart_tmc);

    s_huart_cli.Instance          = CLI_UART;
    s_huart_cli.Init.BaudRate     = CLI_UART_BAUDRATE;
    s_huart_cli.Init.WordLength   = UART_WORDLENGTH_8B;
    s_huart_cli.Init.StopBits     = UART_STOPBITS_1;
    s_huart_cli.Init.Parity       = UART_PARITY_NONE;
    s_huart_cli.Init.Mode         = UART_MODE_TX_RX;
    s_huart_cli.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_huart_cli.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_huart_cli);
}

/* ==== Инициализация драйвера TMC2209 ==== */

static void app_init_driver(void)
{
    tmc2209_config_t cfg = TMC2209_DEFAULT_CONFIG;
    cfg.addr           = TMC2209_UART_ADDR;
    cfg.rsense         = TMC2209_RSENSE_OHM;
    cfg.irun_ma        = TMC2209_IRUN_MA;
    cfg.ihold_ma       = TMC2209_IHOLD_MA;
    cfg.microsteps     = TMC2209_MICROSTEPS;
    cfg.reply_delay_us = TMC_REPLY_DELAY_US;

    tmc2209_io_t io;
    tmc2209_port_stm32_hal_fill_io(&io, &s_hal_ctx);

    tmc2209_result_t res = tmc2209_init(&s_drv, &cfg, &io);
    if (res == TMC2209_OK) {
        tmc2209_enable(&s_drv);
        s_drv_ready = 1;
        s_mode = APP_MODE_UART;
        tx("init ok\r\n");
    } else {
        s_drv_ready = 0;
        snprintf(rsp, sizeof(rsp), "init FAIL: %s\r\n", tmc2209_result_str(res));
        tx(rsp);
    }
}

/* ==== Настройка тактирования (HSI, внешний кварц не используется) ==== */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;  /* HSI/2 = 4 МГц */
    osc.PLL.PLLMUL          = RCC_PLL_MUL12;           /* 4×12 = 48 МГц */
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1);
}

/* ==== Обработчики прерываний ==== */

void SysTick_Handler(void) { HAL_IncTick(); }

/* ==== Точка входа main ==== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    tmc2209_port_stm32_dwt_init();
    gpio_hw_init();
    uart_hw_init();
    step_tim_init();

    s_hal_ctx.huart       = &s_huart_tmc;
    s_hal_ctx.en_port     = ENABLE_PORT;
    s_hal_ctx.en_pin      = ENABLE_PIN;
    s_hal_ctx.sysclk_hz   = SYSCLK_HZ;
    s_hal_ctx.half_duplex = 0;
    s_hal_ctx.debug_fn    = debug_to_cli;

    tx("Test_TMC2209 F103 Start\r\n");

    HAL_Delay(TMC2209_POWERON_DELAY_MS);
    app_init_driver();
    if (!s_drv_ready) {
        tx("retry init...\r\n");
        HAL_Delay(TMC2209_INIT_RETRY_DELAY_MS);
        app_init_driver();
    }

    uint32_t last_cs_tick = 0;

    while (1) {
        if (s_continuous_cs) {
            uint32_t now = HAL_GetTick();
            if ((uint32_t)(now - last_cs_tick) >= CS_INTERVAL_MS) {
                last_cs_tick = now;
                tmc2209_drv_status_t ds;
                tmc2209_result_t r = tmc2209_get_drv_status(&s_drv, &ds);
                snprintf(rsp, sizeof(rsp), "cs:%u%s\r\n",
                         (r == TMC2209_OK) ? ds.cs_actual : 0,
                         (r == TMC2209_OK) ? "" : " ?");
                tx(rsp);
            }
        }
        process_cmd();
    }
}
