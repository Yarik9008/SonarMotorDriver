/* tmc2209.c — TMC2209 драйвер (STM32F446).
 *
 * Два режима управления:
 *   UART     — встроенный генератор шагов (VACTUAL)
 *   STEP/DIR — аппаратные импульсы TIM4 PWM CH3 (PB8)
 *
 * На основе FW_SonarMotorDriver.
 */

#include "tmc2209.h"

/* ---- Register addresses ---- */
#define TMC_REG_GCONF       0x00U
#define TMC_REG_GSTAT       0x01U
#define TMC_REG_SLAVECONF   0x03U
#define TMC_REG_IFCNT       0x02U
#define TMC_REG_IOIN        0x06U
#define TMC_REG_IHOLD_IRUN  0x10U
#define TMC_REG_TPOWERDOWN  0x11U
#define TMC_REG_TPWMTHRS    0x13U
#define TMC_REG_VACTUAL     0x22U
#define TMC_REG_SGTHRS      0x40U
#define TMC_REG_SG_RESULT   0x41U
#define TMC_REG_CHOPCONF    0x6CU
#define TMC_REG_DRV_STATUS  0x6FU
#define TMC_REG_PWMCONF     0x70U

/* ---- Bit-field helpers ---- */
#define GCONF_PDN_DISABLE       (1U << 6)
#define GCONF_MSTEP_REG_SELECT  (1U << 7)
#define GCONF_MULTISTEP_FILT    (1U << 8)

#define IHOLD_IRUN_IHOLD(n)      ((uint32_t)((n) & 0x1FU) << 0)
#define IHOLD_IRUN_IRUN(n)       ((uint32_t)((n) & 0x1FU) << 8)
#define IHOLD_IRUN_IHOLDDELAY(n) ((uint32_t)((n) & 0x0FU) << 16)

#define CHOPCONF_MRES_MASK       (0x0FUL << 24)
#define CHOPCONF_MRES(n)         ((uint32_t)((n) & 0x0FU) << 24)

#define TMC_SYNC            0x05U
#define TMC_WRITE_BIT       0x80U
#define TMC_SENDDELAY_US    100U

#define DWT_CYCLES(us)      ((uint32_t)(us) * (SYSCLK_HZ / 1000000U))

/* counter freq after prescaler */
#define STEP_CNT_HZ         (STEP_TIM_CLK_HZ / (STEP_TIM_PSC + 1U))

static UART_HandleTypeDef huart_tmc;
static TIM_HandleTypeDef  htim_step;
static TMC2209_Mode       s_mode = TMC_MODE_UART;
static uint8_t            s_tim_running;
static void (*s_debug_print)(const char *str);

void TMC2209_SetDebugPrint(void (*fn)(const char *str))
{
    s_debug_print = fn;
}

/* ---- DWT cycle counter ---- */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ---- CRC8-ATM (полином 0x07, LSB first) ---- */

static void tmc_crc(uint8_t *dg, uint8_t len)
{
    uint8_t *c = dg + (len - 1);
    *c = 0;
    for (uint8_t i = 0; i < len - 1; i++) {
        uint8_t b = dg[i];
        for (uint8_t j = 0; j < 8; j++) {
            *c = ((*c >> 7) ^ (b & 1)) ? (*c << 1) ^ 0x07 : (*c << 1);
            b >>= 1;
        }
    }
}

/* ---- Helpers ---- */

static uint8_t current_to_cs(uint32_t ma, float rsense)
{
    float ifs = 0.325f / ((rsense + 0.02f) * 1.414f);
    float cs  = (float)ma / 1000.0f / ifs * 32.0f - 1.0f;
    if (cs < 0)  return 0;
    if (cs > 31) return 31;
    return (uint8_t)(cs + 0.5f);
}

static uint8_t microsteps_to_mres(uint32_t ms)
{
    switch (ms) {
        case 256: return 0;  case 128: return 1;  case 64: return 2;
        case 32:  return 3;  case 16:  return 4;  case 8:  return 5;
        case 4:   return 6;  case 2:   return 7;  case 1:  return 8;
        default:  return 3;
    }
}

/* ---- HAL UART MspInit for USART2 (TMC2209) ---- */

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    if (h->Instance != TMC2209_UART) return;

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Half-duplex single-wire: open-drain TX (release bus for TMC reply) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = TMC2209_UART_TX_PIN;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(TMC2209_UART_TX_PORT, &gpio);

    gpio.Pin       = TMC2209_UART_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(TMC2209_UART_RX_PORT, &gpio);

    HAL_NVIC_SetPriority(USART2_IRQn, IRQ_PRIO_TMC_UART, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *h)
{
    if (h->Instance != TMC2209_UART) return;
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(TMC2209_UART_TX_PORT, TMC2209_UART_TX_PIN);
    HAL_GPIO_DeInit(TMC2209_UART_RX_PORT, TMC2209_UART_RX_PIN);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
}

/* ---- Blocking UART write/read ---- */

static uint8_t tmc_write(uint8_t reg, uint32_t val)
{
    uint8_t buf[8] = { TMC_SYNC, TMC2209_UART_ADDR, reg | TMC_WRITE_BIT,
                       val >> 24, val >> 16, val >> 8, val, 0 };
    tmc_crc(buf, 8);
    HAL_HalfDuplex_EnableTransmitter(&huart_tmc);
    HAL_StatusTypeDef st = HAL_UART_Transmit(&huart_tmc, buf, 8, 20);
    HAL_HalfDuplex_EnableReceiver(&huart_tmc);
    return (st == HAL_OK) ? 1 : 0;
}

static uint8_t tmc_read(uint8_t reg, uint32_t *out)
{
    uint8_t req[4] = { TMC_SYNC, TMC2209_UART_ADDR, reg, 0 };
    tmc_crc(req, 4);
    HAL_HalfDuplex_EnableTransmitter(&huart_tmc);
    if (HAL_UART_Transmit(&huart_tmc, req, 4, 20) != HAL_OK) {
        HAL_HalfDuplex_EnableReceiver(&huart_tmc);
        return 0;
    }
    HAL_HalfDuplex_EnableReceiver(&huart_tmc);

    uint32_t t0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - t0) < DWT_CYCLES(TMC_SENDDELAY_US)) {}

    uint8_t rsp[8];
    if (HAL_UART_Receive(&huart_tmc, rsp, 8, 20) != HAL_OK)
        return 0;

    if (rsp[0] != TMC_SYNC || rsp[1] != 0xFF || rsp[2] != reg)
        return 0;

    uint8_t crc = rsp[7];
    rsp[7] = 0;
    tmc_crc(rsp, 8);
    if (crc != rsp[7])
        return 0;

    *out = ((uint32_t)rsp[3] << 24) | ((uint32_t)rsp[4] << 16) |
           ((uint32_t)rsp[5] << 8)  |  (uint32_t)rsp[6];
    return 1;
}

/* ---- TIM4 PWM for STEP pulses ---- */

static void step_tim_init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim_step.Instance               = STEP_TIM;
    htim_step.Init.Prescaler         = STEP_TIM_PSC;
    htim_step.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim_step.Init.Period            = 0xFFFF;
    htim_step.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_step.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim_step);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = STEP_TIM_PULSE_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim_step, &oc, STEP_TIM_CH);

    s_tim_running = 0;
}

static void step_pin_as_gpio(void)
{
    if (s_tim_running) {
        HAL_TIM_PWM_Stop(&htim_step, STEP_TIM_CH);
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
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = STEP_TIM_AF;
    HAL_GPIO_Init(STEP_PORT, &gpio);
}

static void step_pwm_start(uint32_t steps_per_sec)
{
    if (steps_per_sec == 0) {
        if (s_tim_running) {
            HAL_TIM_PWM_Stop(&htim_step, STEP_TIM_CH);
            s_tim_running = 0;
        }
        return;
    }
    uint32_t arr = (STEP_CNT_HZ / steps_per_sec);
    if (arr < 4) arr = 4;
    if (arr > 0xFFFF) arr = 0xFFFF;
    __HAL_TIM_SET_AUTORELOAD(&htim_step, arr - 1);
    __HAL_TIM_SET_COMPARE(&htim_step, STEP_TIM_CH, STEP_TIM_PULSE_US);
    if (!s_tim_running) {
        step_pin_as_pwm();
        HAL_TIM_PWM_Start(&htim_step, STEP_TIM_CH);
        s_tim_running = 1;
    }
}

static void step_pwm_stop(void)
{
    if (s_tim_running) {
        HAL_TIM_PWM_Stop(&htim_step, STEP_TIM_CH);
        s_tim_running = 0;
    }
    step_pin_as_gpio();
}

/* ---- Non-blocking init state machine ---- */

#define SLAVECONF_SENDDELAY(n) ((uint32_t)((n) & 0x0FU) << 0)

typedef enum {
    TI_DWT, TI_GPIO, TI_UART, TI_TIM, TI_POWERUP,
    TI_GCONF, TI_IHOLD_IRUN, TI_TPWMTHRS, TI_SLAVECONF,
    TI_CHOP_RD, TI_CHOP_WR,
    TI_PWMCONF, TI_ENABLE,
    TI_DONE, TI_ERR
} TIState;

static TIState        s_ti;
static uint32_t       s_ti_t0;
static uint32_t       s_chop;
static TMC2209_Status s_result;

void TMC2209_InitStart(void)
{
    s_ti     = TI_DWT;
    s_result = TMC_BUSY;
}

TMC2209_Status TMC2209_Poll(void)
{
    if (s_result != TMC_BUSY)
        return s_result;

    switch (s_ti) {

    case TI_DWT:
        dwt_init();
        s_ti = TI_GPIO;
        break;

    case TI_GPIO: {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, GPIO_PIN_SET);
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = ENABLE_PIN;
        gpio.Mode  = GPIO_MODE_OUTPUT_PP;
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(ENABLE_PORT, &gpio);

        gpio.Pin = DIR_PIN;
        HAL_GPIO_Init(DIR_PORT, &gpio);

        gpio.Pin = STEP_PIN;
        HAL_GPIO_Init(STEP_PORT, &gpio);

        s_ti = TI_UART;
        break;
    }

    case TI_UART:
        huart_tmc.Instance          = TMC2209_UART;
        huart_tmc.Init.BaudRate     = TMC2209_UART_BAUDRATE;
        huart_tmc.Init.WordLength   = UART_WORDLENGTH_8B;
        huart_tmc.Init.StopBits     = UART_STOPBITS_1;
        huart_tmc.Init.Parity       = UART_PARITY_NONE;
        huart_tmc.Init.Mode         = UART_MODE_TX_RX;
        huart_tmc.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
        huart_tmc.Init.OverSampling = UART_OVERSAMPLING_16;
        HAL_HalfDuplex_Init(&huart_tmc);
        s_ti = TI_TIM;
        break;

    case TI_TIM:
        step_tim_init();
        s_ti_t0 = DWT->CYCCNT;
        s_ti = TI_POWERUP;
        break;

    case TI_POWERUP:
        if ((DWT->CYCCNT - s_ti_t0) >= DWT_CYCLES(10000U))
            s_ti = TI_GCONF;
        break;

    case TI_GCONF:
        if (s_debug_print)
            s_debug_print("--- TMC2209 UART init ---\r\n");
        tmc_write(TMC_REG_GCONF, GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT | GCONF_MULTISTEP_FILT);
        s_ti = TI_IHOLD_IRUN;
        break;

    case TI_IHOLD_IRUN: {
        uint8_t irun  = current_to_cs(TMC2209_IRUN_MA,  TMC2209_RSENSE_OHM);
        uint8_t ihold = current_to_cs(TMC2209_IHOLD_MA, TMC2209_RSENSE_OHM);
        if (ihold > irun) ihold = irun;
        tmc_write(TMC_REG_IHOLD_IRUN,
            IHOLD_IRUN_IHOLD(ihold) | IHOLD_IRUN_IRUN(irun) | IHOLD_IRUN_IHOLDDELAY(4));
        s_ti = TI_TPWMTHRS;
        break;
    }

    case TI_TPWMTHRS:
        tmc_write(TMC_REG_TPWMTHRS, 0);
        s_ti = TI_SLAVECONF;
        break;

    case TI_SLAVECONF:
        tmc_write(TMC_REG_SLAVECONF, SLAVECONF_SENDDELAY(4));
        s_ti = TI_CHOP_RD;
        break;

    case TI_CHOP_RD: {
        uint32_t v = 0;
        if (tmc_read(TMC_REG_CHOPCONF, &v))
            s_chop = (v & ~CHOPCONF_MRES_MASK) | CHOPCONF_MRES(microsteps_to_mres(TMC2209_MICROSTEPS));
        else
            s_chop = CHOPCONF_MRES(microsteps_to_mres(TMC2209_MICROSTEPS));
        s_ti = TI_CHOP_WR;
        break;
    }

    case TI_CHOP_WR:
        tmc_write(TMC_REG_CHOPCONF, s_chop);
        s_ti = TI_PWMCONF;
        break;

    case TI_PWMCONF:
        tmc_write(TMC_REG_PWMCONF, 0xC10D0024U);
        s_ti = TI_ENABLE;
        break;

    case TI_ENABLE:
        HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, GPIO_PIN_RESET);
        tmc_write(TMC_REG_VACTUAL, 0);
        s_mode = TMC_MODE_UART;
        s_result = TMC_DONE;
        break;

    case TI_DONE:
        s_result = TMC_DONE;
        break;

    case TI_ERR:
        s_result = TMC_ERROR;
        break;
    }

    return s_result;
}

/* ---- Public API ---- */

void TMC2209_SetMode(TMC2209_Mode mode)
{
    if (mode == s_mode) return;

    if (mode == TMC_MODE_STEP_DIR) {
        tmc_write(TMC_REG_VACTUAL, 0);
        s_mode = TMC_MODE_STEP_DIR;
    } else {
        step_pwm_stop();
        s_mode = TMC_MODE_UART;
    }
}

TMC2209_Mode TMC2209_GetMode(void)
{
    return s_mode;
}

void TMC2209_Move(int32_t value)
{
    if (s_mode == TMC_MODE_UART) {
        tmc_write(TMC_REG_VACTUAL, (uint32_t)value);
    } else {
        if (value == 0) {
            step_pwm_stop();
            return;
        }
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, (value > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        uint32_t abs_val = (value > 0) ? (uint32_t)value : (uint32_t)(-value);
        step_pwm_start(abs_val);
    }
}

void TMC2209_Stop(void)
{
    if (s_mode == TMC_MODE_UART) {
        tmc_write(TMC_REG_VACTUAL, 0);
    } else {
        step_pwm_stop();
    }
}

void TMC2209_SetEnable(uint8_t en)
{
    HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

uint8_t TMC2209_ReadVersion(void)
{
    uint32_t v = 0;
    if (!tmc_read(TMC_REG_IOIN, &v))
        return 0;
    return (v >> 24) & 0xFF;
}

uint32_t TMC2209_ReadDrvStatus(void)
{
    uint32_t v = 0;
    tmc_read(TMC_REG_DRV_STATUS, &v);
    return v;
}

uint16_t TMC2209_ReadSgResult(void)
{
    uint32_t v = 0;
    tmc_read(TMC_REG_SG_RESULT, &v);
    return (uint16_t)(v & 0x3FF);
}

uint8_t TMC2209_ReadReg(uint8_t reg, uint32_t *out)
{
    return tmc_read(reg, out);
}

/* ---- USART2 IRQ ---- */

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart_tmc);
}
