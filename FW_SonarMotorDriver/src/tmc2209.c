/* tmc2209.c — Неблокирующий UART-драйвер TMC2209 (single-wire half-duplex). */

#include "tmc2209.h"
#include "board.h"

#define TMC_REG_GCONF       0x00U
#define TMC_REG_IHOLD_IRUN  0x10U
#define TMC_REG_CHOPCONF    0x18U

#define GCONF_PDN_DISABLE       (1U << 6)
#define GCONF_MSTEP_REG_SELECT  (1U << 7)

#define IHOLD_IRUN_IHOLD(n)      ((uint32_t)((n) & 0x1FU) << 0)
#define IHOLD_IRUN_IRUN(n)       ((uint32_t)((n) & 0x1FU) << 8)
#define IHOLD_IRUN_IHOLDDELAY(n) ((uint32_t)((n) & 0x0FU) << 16)

#define CHOPCONF_MRES(n)    ((uint32_t)((n) & 0x0FU) << 24)

#define TMC_SYNC            0x05U
#define TMC_WRITE_BIT       0x80U
#define TMC_SENDDELAY_US    100U
#define TMC_TX_TIMEOUT_US   3000U
#define TMC_RX_TIMEOUT_US   4000U
#define DWT_CYCLES(us)      ((uint32_t)(us) * (SYSCLK_HZ / 1000000U))

static UART_HandleTypeDef huart_tmc;

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

/* ---- Вспомогательные ---- */

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

/* ---- Блокирующая запись (короткая, ~700 мкс) ---- */

static uint8_t tmc_write(uint8_t reg, uint32_t val)
{
    uint8_t buf[8] = { TMC_SYNC, TMC2209_UART_ADDR, reg | TMC_WRITE_BIT,
                       val >> 24, val >> 16, val >> 8, val, 0 };
    tmc_crc(buf, 8);
    return (HAL_UART_Transmit(&huart_tmc, buf, 8, 20) == HAL_OK) ? 1 : 0;
}

/* ---- Неблокирующее чтение регистра (IRQ state machine) ---- */

typedef enum {
    RD_IDLE = 0, RD_TX, RD_SENDDELAY, RD_RX, RD_OK, RD_FAIL
} RdState;

static volatile uint8_t s_tx_done, s_rx_done, s_uart_err;
static RdState  s_rd;
static uint8_t  s_req[4], s_rsp[8], s_reg;
static uint32_t s_t0, s_val;

static uint8_t rd_start(uint8_t reg)
{
    s_reg = reg;
    s_req[0] = TMC_SYNC;
    s_req[1] = TMC2209_UART_ADDR;
    s_req[2] = reg;
    tmc_crc(s_req, 4);

    s_tx_done = s_rx_done = s_uart_err = 0;
    s_t0 = DWT->CYCCNT;

    if (HAL_UART_Transmit_IT(&huart_tmc, s_req, 4) != HAL_OK) {
        s_rd = RD_FAIL;
        return 0;
    }
    s_rd = RD_TX;
    return 1;
}

/* 0 = busy, 1 = ok, 2 = error */
static uint8_t rd_poll(uint32_t *out)
{
    switch (s_rd) {
    case RD_IDLE:
        return 2;

    case RD_TX:
        if (s_uart_err || (DWT->CYCCNT - s_t0) >= DWT_CYCLES(TMC_TX_TIMEOUT_US))
            return (s_rd = RD_FAIL), 2;
        if (!s_tx_done)
            return 0;
        s_t0 = DWT->CYCCNT;
        s_rd = RD_SENDDELAY;
        return 0;

    case RD_SENDDELAY:
        if ((DWT->CYCCNT - s_t0) < DWT_CYCLES(TMC_SENDDELAY_US))
            return 0;
        s_rx_done = s_uart_err = 0;
        s_t0 = DWT->CYCCNT;
        if (HAL_UART_Receive_IT(&huart_tmc, s_rsp, 8) != HAL_OK)
            return (s_rd = RD_FAIL), 2;
        s_rd = RD_RX;
        return 0;

    case RD_RX:
        if (s_uart_err || (DWT->CYCCNT - s_t0) >= DWT_CYCLES(TMC_RX_TIMEOUT_US))
            return (s_rd = RD_FAIL), 2;
        if (!s_rx_done)
            return 0;
        if (s_rsp[0] != TMC_SYNC || s_rsp[1] != TMC2209_UART_ADDR || s_rsp[2] != s_reg)
            return (s_rd = RD_FAIL), 2;
        {
            uint8_t crc = s_rsp[7];
            s_rsp[7] = 0;
            tmc_crc(s_rsp, 8);
            if (crc != s_rsp[7])
                return (s_rd = RD_FAIL), 2;
        }
        s_val = ((uint32_t)s_rsp[3] << 24) | ((uint32_t)s_rsp[4] << 16) |
                ((uint32_t)s_rsp[5] << 8)  |  (uint32_t)s_rsp[6];
        if (out) *out = s_val;
        s_rd = RD_OK;
        return 1;

    case RD_OK:
        if (out) *out = s_val;
        return 1;

    default:
        return 2;
    }
}

/* ---- Стейт-машина инициализации ---- */

typedef enum {
    TI_UART, TI_POWERUP, TI_GCONF, TI_IHOLD_IRUN,
    TI_CHOP_RD_START, TI_CHOP_RD_WAIT, TI_CHOP_WR,
    TI_DONE, TI_ERR
} TIState;

static TIState        s_ti;
static uint32_t       s_ti_t0;
static uint32_t       s_chop;
static TMC2209_Status s_result;

void TMC2209_InitStart(void)
{
    s_ti     = TI_UART;
    s_result = TMC_BUSY;
}

TMC2209_Status TMC2209_Poll(void)
{
    if (s_result != TMC_BUSY)
        return s_result;

    switch (s_ti) {

    case TI_UART:
        huart_tmc.Instance          = TMC2209_UART;
        huart_tmc.Init.BaudRate     = TMC2209_UART_BAUDRATE;
        huart_tmc.Init.WordLength   = UART_WORDLENGTH_8B;
        huart_tmc.Init.StopBits     = UART_STOPBITS_1;
        huart_tmc.Init.Parity       = UART_PARITY_NONE;
        huart_tmc.Init.Mode         = UART_MODE_TX_RX;
        huart_tmc.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
        huart_tmc.Init.OverSampling = UART_OVERSAMPLING_16;
        HAL_UART_Init(&huart_tmc);
        s_ti_t0 = DWT->CYCCNT;
        s_ti = TI_POWERUP;
        break;

    case TI_POWERUP:
        if ((DWT->CYCCNT - s_ti_t0) >= DWT_CYCLES(10000))
            s_ti = TI_GCONF;
        break;

    case TI_GCONF:
        tmc_write(TMC_REG_GCONF, GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT);
        s_ti = TI_IHOLD_IRUN;
        break;

    case TI_IHOLD_IRUN: {
        uint8_t irun  = current_to_cs(TMC2209_IRUN_MA,  TMC2209_RSENSE_OHM);
        uint8_t ihold = current_to_cs(TMC2209_IHOLD_MA, TMC2209_RSENSE_OHM);
        if (ihold > irun) ihold = irun;
        tmc_write(TMC_REG_IHOLD_IRUN,
            IHOLD_IRUN_IHOLD(ihold) | IHOLD_IRUN_IRUN(irun) | IHOLD_IRUN_IHOLDDELAY(4));
        s_ti = TI_CHOP_RD_START;
        break;
    }

    case TI_CHOP_RD_START:
        if (!rd_start(TMC_REG_CHOPCONF))
            s_chop = CHOPCONF_MRES(microsteps_to_mres(TMC2209_MICROSTEPS));
        s_ti = TI_CHOP_RD_WAIT;
        break;

    case TI_CHOP_RD_WAIT: {
        uint32_t v = 0;
        uint8_t r = rd_poll(&v);
        if (r == 0) break;
        s_chop = (r == 1)
            ? (v & ~(0x0FU << 24)) | CHOPCONF_MRES(microsteps_to_mres(TMC2209_MICROSTEPS))
            : CHOPCONF_MRES(microsteps_to_mres(TMC2209_MICROSTEPS));
        s_ti = TI_CHOP_WR;
        break;
    }

    case TI_CHOP_WR:
        tmc_write(TMC_REG_CHOPCONF, s_chop);
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

/* ---- IRQ glue ---- */

void TMC2209_UartIrqHandler(void) { HAL_UART_IRQHandler(&huart_tmc); }

void TMC2209_UartTxCpltCb(UART_HandleTypeDef *h)
{
    if (h->Instance == TMC2209_UART) s_tx_done = 1;
}

void TMC2209_UartRxCpltCb(UART_HandleTypeDef *h)
{
    if (h->Instance == TMC2209_UART) s_rx_done = 1;
}

void TMC2209_UartErrorCb(UART_HandleTypeDef *h)
{
    if (h->Instance == TMC2209_UART) s_uart_err = 1;
}
