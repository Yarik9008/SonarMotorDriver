/* tmc2209.c — App wrapper: TMC2209 library integration for SonarMotorDriver.
 *
 * Owns tmc2209_t, UART handle, HAL context.
 * Exposes simple functions for main.c; keeps library details internal.
 *
 * UART mode: full-duplex (TX PA2 through 1 kΩ, RX PA3 direct to PDN_UART).
 * Enable pin: shared with stepper.c — both write to ENABLE_PORT/ENABLE_PIN.
 */

#include "tmc2209.h"
#include "tmc2209_port_stm32_hal.h"
#include "board.h"

/* ---- Internal state ---- */

static tmc2209_t          s_drv;
static UART_HandleTypeDef s_huart;
static tmc2209_hal_ctx_t  s_hal;

typedef enum { ST_IDLE = 0, ST_PENDING, ST_DONE, ST_ERROR } WrapperState;
static WrapperState s_state;

/* ---- Init API ---- */

void TMC2209_InitStart(void)
{
    s_state = ST_PENDING;
}

TMC2209_Status TMC2209_Poll(void)
{
    if (s_state == ST_IDLE)  return TMC_BUSY;
    if (s_state >= ST_DONE)  return (s_state == ST_DONE) ? TMC_DONE : TMC_ERROR;

    /* --- One-shot blocking init --- */

    /* UART peripheral */
    s_huart.Instance          = TMC2209_UART;
    s_huart.Init.BaudRate     = TMC2209_UART_BAUDRATE;
    s_huart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_huart.Init.StopBits     = UART_STOPBITS_1;
    s_huart.Init.Parity       = UART_PARITY_NONE;
    s_huart.Init.Mode         = UART_MODE_TX_RX;
    s_huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_huart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_huart);

    /* HAL port context */
    s_hal.huart       = &s_huart;
    s_hal.en_port     = ENABLE_PORT;
    s_hal.en_pin      = ENABLE_PIN;
    s_hal.sysclk_hz   = SYSCLK_HZ;
    s_hal.half_duplex = TMC2209_HALF_DUPLEX;
    s_hal.debug_fn    = NULL;

    /* I/O callbacks */
    tmc2209_io_t io;
    tmc2209_port_stm32_hal_fill_io(&io, &s_hal);

    /* Driver config from board.h */
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

    /* Blocking library init (configures registers, verifies communication) */
    tmc2209_result_t res = tmc2209_init(&s_drv, &cfg, &io);
    if (res == TMC2209_OK) {
        s_state = ST_DONE;
        return TMC_DONE;
    }

    s_state = ST_ERROR;
    return TMC_ERROR;
}

/* ---- State ---- */

uint8_t TMC2209_IsReady(void)
{
    return s_state == ST_DONE;
}

/* ---- Enable / disable ---- */

void TMC2209_SetEnabled(uint8_t enabled)
{
    if (!TMC2209_IsReady()) return;
    if (enabled)
        tmc2209_enable(&s_drv);
    else
        tmc2209_disable(&s_drv);
}

/* ---- Runtime reconfiguration ---- */

TMC2209_Status TMC2209_SetCurrent(uint16_t run_ma, uint16_t hold_ma)
{
    if (!TMC2209_IsReady()) return TMC_ERROR;
    return (tmc2209_set_current(&s_drv, run_ma, hold_ma) == TMC2209_OK)
        ? TMC_DONE : TMC_ERROR;
}

TMC2209_Status TMC2209_SetMicrosteps(uint16_t ms)
{
    if (!TMC2209_IsReady()) return TMC_ERROR;
    return (tmc2209_set_microsteps(&s_drv, ms) == TMC2209_OK)
        ? TMC_DONE : TMC_ERROR;
}

/* ---- Diagnostics ---- */

TMC2209_Status TMC2209_GetDrvStatus(tmc2209_drv_status_t *st)
{
    if (!TMC2209_IsReady()) return TMC_ERROR;
    return (tmc2209_get_drv_status(&s_drv, st) == TMC2209_OK)
        ? TMC_DONE : TMC_ERROR;
}

TMC2209_Status TMC2209_GetVersion(uint8_t *version)
{
    if (!TMC2209_IsReady()) return TMC_ERROR;
    return (tmc2209_get_version(&s_drv, version) == TMC2209_OK)
        ? TMC_DONE : TMC_ERROR;
}

/* ---- Direct access ---- */

tmc2209_t *TMC2209_GetDriver(void)
{
    return TMC2209_IsReady() ? &s_drv : 0;
}
