/* tmc2209.c — TMC2209 library core implementation.
 *
 * Platform-agnostic: all hardware access goes through tmc2209_io_t callbacks.
 * Contains UART protocol (CRC, framing), register I/O, init sequence,
 * configuration helpers, and diagnostic register decoders.
 */

#include "tmc2209/tmc2209.h"
#include <string.h>
#include <stdio.h>

#define RX_BUF_SIZE 16U

/* ---- Debug print helper ---- */

static char s_dbg[128];

#define DBG(drv, ...) do { \
    if ((drv)->io.debug_print) { \
        snprintf(s_dbg, sizeof(s_dbg), __VA_ARGS__); \
        (drv)->io.debug_print(s_dbg, (drv)->io.ctx); \
    } \
} while (0)

static void dbg_hex(tmc2209_t *drv, const char *prefix,
                    const uint8_t *data, uint16_t len)
{
    if (!drv->io.debug_print) return;
    char buf[80];
    size_t pos = 0;
    pos += (size_t)snprintf(buf + pos, sizeof(buf) - pos, "%s", prefix);
    for (uint16_t i = 0; i < len && pos < sizeof(buf) - 4; i++)
        pos += (size_t)snprintf(buf + pos, sizeof(buf) - pos, "%02X ", data[i]);
    snprintf(buf + pos, sizeof(buf) - pos, "\r\n");
    drv->io.debug_print(buf, drv->io.ctx);
}

/* ---- CRC8-ATM (polynomial 0x07, LSB-first) ---- */

static void tmc_crc(uint8_t *datagram, uint8_t len)
{
    uint8_t *crc = datagram + (len - 1);
    *crc = 0;
    for (uint8_t i = 0; i < len - 1; i++) {
        uint8_t byte = datagram[i];
        for (uint8_t j = 0; j < 8; j++) {
            *crc = ((*crc >> 7) ^ (byte & 1))
                   ? (*crc << 1) ^ 0x07
                   : (*crc << 1);
            byte >>= 1;
        }
    }
}

/* ---- Helpers ---- */

static uint8_t current_to_cs(uint16_t ma, float rsense)
{
    float ifs = 0.325f / ((rsense + 0.02f) * 1.414f);
    float cs  = (float)ma / 1000.0f / ifs * 32.0f - 1.0f;
    if (cs < 0.0f) return 0;
    if (cs > 31.0f) return 31;
    return (uint8_t)(cs + 0.5f);
}

static uint8_t microsteps_to_mres(uint16_t ms)
{
    switch (ms) {
        case 256: return 0;  case 128: return 1;  case 64: return 2;
        case 32:  return 3;  case 16:  return 4;  case 8:  return 5;
        case 4:   return 6;  case 2:   return 7;  case 1:  return 8;
        default:  return 4;
    }
}

/* ==== Low-level register access ==== */

tmc2209_result_t tmc2209_write_reg(tmc2209_t *drv, uint8_t reg, uint32_t value)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;

    uint8_t buf[8] = {
        TMC2209_SYNC_BYTE, drv->cfg.addr, (uint8_t)(reg | TMC2209_WRITE_BIT),
        (uint8_t)(value >> 24), (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),  (uint8_t)(value), 0
    };
    tmc_crc(buf, 8);

    int ret = drv->io.uart_tx(buf, 8, 20, drv->io.ctx);
    if (ret != 0) {
        drv->last_error = TMC2209_ERR_UART;
        return TMC2209_ERR_UART;
    }
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_read_reg_addr(tmc2209_t *drv, uint8_t addr,
                                       uint8_t reg, uint32_t *value)
{
    if (!drv || !value) return TMC2209_ERR_INVALID_ARG;

    drv->io.uart_rx_flush(drv->io.ctx);

    uint8_t req[4] = { TMC2209_SYNC_BYTE, addr, reg, 0 };
    tmc_crc(req, 4);
    dbg_hex(drv, "TX:", req, 4);

    int tx_ret = drv->io.uart_tx(req, 4, 20, drv->io.ctx);
    if (tx_ret != 0) {
        DBG(drv, "TX fail\r\n");
        drv->last_error = TMC2209_ERR_UART;
        return TMC2209_ERR_UART;
    }

    if (drv->cfg.reply_delay_us > 0)
        drv->io.delay_us(drv->cfg.reply_delay_us, drv->io.ctx);

    uint8_t rx_buf[RX_BUF_SIZE];
    memset(rx_buf, 0, sizeof(rx_buf));
    uint16_t n = 0;
    int rx_ret = drv->io.uart_rx(rx_buf, RX_BUF_SIZE, 25, &n, drv->io.ctx);

    DBG(drv, "RX: st=%d n=%u\r\n", rx_ret, (unsigned)n);
    if (n > 0)
        dbg_hex(drv, "RX:", rx_buf, n);

    if (n == 0) {
        DBG(drv, "FAIL: no bytes\r\n");
        drv->last_error = TMC2209_ERR_TIMEOUT;
        return TMC2209_ERR_TIMEOUT;
    }

    for (uint8_t off = 0; off + 8 <= n; off++) {
        uint8_t *r = rx_buf + off;
        if (r[0] != TMC2209_SYNC_BYTE)  continue;
        if (r[1] != TMC2209_MASTER_ADDR) continue;
        if (r[2] != reg)                continue;

        uint8_t crc_saved = r[7];
        r[7] = 0;
        tmc_crc(r, 8);
        if (crc_saved != r[7]) {
            DBG(drv, "FAIL: CRC mismatch @off=%u (got 0x%02X exp 0x%02X)\r\n",
                off, crc_saved, r[7]);
            r[7] = crc_saved;
            continue;
        }

        *value = ((uint32_t)r[3] << 24) | ((uint32_t)r[4] << 16) |
                 ((uint32_t)r[5] << 8)  |  (uint32_t)r[6];
        return TMC2209_OK;
    }

    if (n == 4) {
        uint8_t is_echo = (rx_buf[0] == req[0] && rx_buf[1] == req[1] &&
                           rx_buf[2] == req[2] && rx_buf[3] == req[3]);
        if (is_echo)
            DBG(drv, "FAIL: only echo, no TMC response\r\n");
        else
            DBG(drv, "FAIL: 4 bytes but not echo, no valid frame\r\n");
    } else {
        DBG(drv, "FAIL: no valid frame in %u bytes\r\n", (unsigned)n);
    }

    drv->last_error = TMC2209_ERR_BAD_FRAME;
    return TMC2209_ERR_BAD_FRAME;
}

tmc2209_result_t tmc2209_read_reg(tmc2209_t *drv, uint8_t reg, uint32_t *value)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    return tmc2209_read_reg_addr(drv, drv->cfg.addr, reg, value);
}

/* ==== Init / deinit ==== */

tmc2209_result_t tmc2209_init(tmc2209_t *drv, const tmc2209_config_t *cfg,
                              const tmc2209_io_t *io)
{
    if (!drv || !cfg || !io)           return TMC2209_ERR_INVALID_ARG;
    if (!io->uart_tx || !io->uart_rx)  return TMC2209_ERR_INVALID_ARG;
    if (!io->uart_rx_flush)            return TMC2209_ERR_INVALID_ARG;
    if (!io->delay_us || !io->set_enable) return TMC2209_ERR_INVALID_ARG;

    memset(drv, 0, sizeof(*drv));
    drv->cfg = *cfg;
    drv->io  = *io;

    drv->io.set_enable(1, drv->io.ctx);
    drv->io.delay_us(10000, drv->io.ctx);

    DBG(drv, "--- TMC2209 init ---\r\n");
    DBG(drv, "addr=%u, delay=%uus\r\n",
        (unsigned)cfg->addr, (unsigned)cfg->reply_delay_us);

    tmc2209_result_t res;

    /* GCONF: enable UART control, register-based microsteps, multistep filter */
    res = tmc2209_write_reg(drv, TMC2209_REG_GCONF,
              TMC2209_GCONF_PDN_DISABLE |
              TMC2209_GCONF_MSTEP_REG_SELECT |
              TMC2209_GCONF_MULTISTEP_FILT);
    if (res != TMC2209_OK) return res;

    /* IHOLD_IRUN: motor currents */
    uint8_t irun  = current_to_cs(cfg->irun_ma, cfg->rsense);
    uint8_t ihold = current_to_cs(cfg->ihold_ma, cfg->rsense);
    if (ihold > irun) ihold = irun;
    res = tmc2209_write_reg(drv, TMC2209_REG_IHOLD_IRUN,
              TMC2209_IHOLD(ihold) | TMC2209_IRUN(irun) |
              TMC2209_IHOLDDELAY(cfg->iholddelay));
    if (res != TMC2209_OK) return res;

    /* TPWMTHRS */
    res = tmc2209_write_reg(drv, TMC2209_REG_TPWMTHRS, cfg->tpwmthrs);
    if (res != TMC2209_OK) return res;

    /* SLAVECONF: reply send delay */
    res = tmc2209_write_reg(drv, TMC2209_REG_SLAVECONF,
              TMC2209_SENDDELAY(cfg->senddelay));
    if (res != TMC2209_OK) return res;

    /* Verify communication: read IFCNT */
    uint32_t ifcnt = 0;
    res = tmc2209_read_reg(drv, TMC2209_REG_IFCNT, &ifcnt);
    if (res != TMC2209_OK) {
        DBG(drv, "init ERR: IFCNT read fail\r\n");
        return res;
    }
    DBG(drv, "init: IFCNT=%lu\r\n", (unsigned long)(ifcnt & 0xFF));

    /* Verify VERSION via IOIN */
    uint32_t ioin_raw = 0;
    res = tmc2209_read_reg(drv, TMC2209_REG_IOIN, &ioin_raw);
    if (res != TMC2209_OK) {
        DBG(drv, "init ERR: IOIN read fail\r\n");
        return res;
    }
    uint8_t ver = (uint8_t)((ioin_raw >> TMC2209_IOIN_VERSION_Pos) & 0xFF);
    DBG(drv, "init: IOIN=0x%08lX VERSION=0x%02X\r\n",
        (unsigned long)ioin_raw, ver);
    if (ver != TMC2209_VERSION_EXPECTED) {
        DBG(drv, "init ERR: VERSION 0x%02X != 0x%02X\r\n",
            ver, TMC2209_VERSION_EXPECTED);
        drv->last_error = TMC2209_ERR_HW;
        return TMC2209_ERR_HW;
    }

    /* CHOPCONF: read-modify-write to set microstep resolution */
    uint32_t chop = 0;
    if (tmc2209_read_reg(drv, TMC2209_REG_CHOPCONF, &chop) != TMC2209_OK)
        chop = TMC2209_CHOPCONF_DEFAULT;
    chop = (chop & ~TMC2209_CHOPCONF_MRES_Msk) |
           TMC2209_CHOPCONF_MRES(microsteps_to_mres(cfg->microsteps));
    res = tmc2209_write_reg(drv, TMC2209_REG_CHOPCONF, chop);
    if (res != TMC2209_OK) return res;

    /* PWMCONF */
    uint32_t pwm = cfg->pwmconf ? cfg->pwmconf : 0xC10D0024U;
    res = tmc2209_write_reg(drv, TMC2209_REG_PWMCONF, pwm);
    if (res != TMC2209_OK) return res;

    /* Safe initial state: VACTUAL = 0 */
    res = tmc2209_write_reg(drv, TMC2209_REG_VACTUAL, 0);
    if (res != TMC2209_OK) return res;

    drv->initialized = 1;
    return TMC2209_OK;
}

void tmc2209_deinit(tmc2209_t *drv)
{
    if (!drv) return;
    if (drv->initialized) {
        tmc2209_write_reg(drv, TMC2209_REG_VACTUAL, 0);
        drv->io.set_enable(1, drv->io.ctx);
    }
    drv->initialized = 0;
}

/* ==== Configuration ==== */

tmc2209_result_t tmc2209_set_current(tmc2209_t *drv, uint16_t run_ma,
                                     uint16_t hold_ma)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    uint8_t irun  = current_to_cs(run_ma, drv->cfg.rsense);
    uint8_t ihold = current_to_cs(hold_ma, drv->cfg.rsense);
    if (ihold > irun) ihold = irun;
    drv->cfg.irun_ma  = run_ma;
    drv->cfg.ihold_ma = hold_ma;
    return tmc2209_write_reg(drv, TMC2209_REG_IHOLD_IRUN,
               TMC2209_IHOLD(ihold) | TMC2209_IRUN(irun) |
               TMC2209_IHOLDDELAY(drv->cfg.iholddelay));
}

tmc2209_result_t tmc2209_set_microsteps(tmc2209_t *drv, uint16_t ms)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    uint32_t chop = 0;
    if (tmc2209_read_reg(drv, TMC2209_REG_CHOPCONF, &chop) != TMC2209_OK)
        chop = TMC2209_CHOPCONF_DEFAULT;
    chop = (chop & ~TMC2209_CHOPCONF_MRES_Msk) |
           TMC2209_CHOPCONF_MRES(microsteps_to_mres(ms));
    drv->cfg.microsteps = ms;
    return tmc2209_write_reg(drv, TMC2209_REG_CHOPCONF, chop);
}

tmc2209_result_t tmc2209_set_chopconf(tmc2209_t *drv, uint32_t value)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    return tmc2209_write_reg(drv, TMC2209_REG_CHOPCONF, value);
}

tmc2209_result_t tmc2209_set_pwmconf(tmc2209_t *drv, uint32_t value)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    return tmc2209_write_reg(drv, TMC2209_REG_PWMCONF, value);
}

/* ==== Motor control ==== */

tmc2209_result_t tmc2209_enable(tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    drv->io.set_enable(0, drv->io.ctx);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_disable(tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    drv->io.set_enable(1, drv->io.ctx);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_set_vactual(tmc2209_t *drv, int32_t velocity)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    return tmc2209_write_reg(drv, TMC2209_REG_VACTUAL, (uint32_t)velocity);
}

tmc2209_result_t tmc2209_stop(tmc2209_t *drv)
{
    return tmc2209_set_vactual(drv, 0);
}

/* ==== Diagnostics ==== */

tmc2209_result_t tmc2209_get_version(tmc2209_t *drv, uint8_t *version)
{
    if (!drv || !version) return TMC2209_ERR_INVALID_ARG;
    uint32_t v = 0;
    tmc2209_result_t res = tmc2209_read_reg(drv, TMC2209_REG_IOIN, &v);
    if (res != TMC2209_OK) return res;
    *version = (uint8_t)((v >> TMC2209_IOIN_VERSION_Pos) & 0xFF);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_ifcnt(tmc2209_t *drv, uint8_t *count)
{
    if (!drv || !count) return TMC2209_ERR_INVALID_ARG;
    uint32_t v = 0;
    tmc2209_result_t res = tmc2209_read_reg(drv, TMC2209_REG_IFCNT, &v);
    if (res != TMC2209_OK) return res;
    *count = (uint8_t)(v & 0xFF);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_ioin(tmc2209_t *drv, tmc2209_ioin_t *ioin)
{
    if (!drv || !ioin) return TMC2209_ERR_INVALID_ARG;
    uint32_t v = 0;
    tmc2209_result_t res = tmc2209_read_reg(drv, TMC2209_REG_IOIN, &v);
    if (res != TMC2209_OK) return res;
    ioin->enn       = (v >> TMC2209_IOIN_ENN_Pos)       & 1;
    ioin->ms1       = (v >> TMC2209_IOIN_MS1_Pos)       & 1;
    ioin->ms2       = (v >> TMC2209_IOIN_MS2_Pos)       & 1;
    ioin->diag      = (v >> TMC2209_IOIN_DIAG_Pos)      & 1;
    ioin->pdn_uart  = (v >> TMC2209_IOIN_PDN_UART_Pos)  & 1;
    ioin->step      = (v >> TMC2209_IOIN_STEP_Pos)      & 1;
    ioin->spread_en = (v >> TMC2209_IOIN_SPREAD_EN_Pos)  & 1;
    ioin->dir       = (v >> TMC2209_IOIN_DIR_Pos)       & 1;
    ioin->version   = (uint8_t)((v >> TMC2209_IOIN_VERSION_Pos) & 0xFF);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_drv_status(tmc2209_t *drv, tmc2209_drv_status_t *status)
{
    if (!drv || !status) return TMC2209_ERR_INVALID_ARG;
    uint32_t v = 0;
    tmc2209_result_t res = tmc2209_read_reg(drv, TMC2209_REG_DRV_STATUS, &v);
    if (res != TMC2209_OK) return res;
    status->otpw      = (v >> TMC2209_DRV_OTPW_Pos)  & 1;
    status->ot        = (v >> TMC2209_DRV_OT_Pos)    & 1;
    status->s2ga      = (v >> TMC2209_DRV_S2GA_Pos)  & 1;
    status->s2gb      = (v >> TMC2209_DRV_S2GB_Pos)  & 1;
    status->s2vsa     = (v >> TMC2209_DRV_S2VSA_Pos) & 1;
    status->s2vsb     = (v >> TMC2209_DRV_S2VSB_Pos) & 1;
    status->ola       = (v >> TMC2209_DRV_OLA_Pos)   & 1;
    status->olb       = (v >> TMC2209_DRV_OLB_Pos)   & 1;
    status->t120      = (v >> TMC2209_DRV_T120_Pos)  & 1;
    status->t143      = (v >> TMC2209_DRV_T143_Pos)  & 1;
    status->t150      = (v >> TMC2209_DRV_T150_Pos)  & 1;
    status->t157      = (v >> TMC2209_DRV_T157_Pos)  & 1;
    status->cs_actual = (uint8_t)((v & TMC2209_DRV_CS_ACTUAL_Msk)
                                   >> TMC2209_DRV_CS_ACTUAL_Pos);
    status->stealth   = (v >> TMC2209_DRV_STEALTH_Pos) & 1;
    status->stst      = (v >> TMC2209_DRV_STST_Pos)   & 1;
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_gstat(tmc2209_t *drv, tmc2209_gstat_t *gstat)
{
    if (!drv || !gstat) return TMC2209_ERR_INVALID_ARG;
    uint32_t v = 0;
    tmc2209_result_t res = tmc2209_read_reg(drv, TMC2209_REG_GSTAT, &v);
    if (res != TMC2209_OK) return res;
    gstat->reset   = (v >> 0) & 1;
    gstat->drv_err = (v >> 1) & 1;
    gstat->uv_cp   = (v >> 2) & 1;
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_sg_result(tmc2209_t *drv, uint16_t *result)
{
    if (!drv || !result) return TMC2209_ERR_INVALID_ARG;
    uint32_t v = 0;
    tmc2209_result_t res = tmc2209_read_reg(drv, TMC2209_REG_SG_RESULT, &v);
    if (res != TMC2209_OK) return res;
    *result = (uint16_t)(v & 0x3FF);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_tstep(tmc2209_t *drv, uint32_t *tstep)
{
    if (!drv || !tstep) return TMC2209_ERR_INVALID_ARG;
    uint32_t v = 0;
    tmc2209_result_t res = tmc2209_read_reg(drv, TMC2209_REG_TSTEP, &v);
    if (res != TMC2209_OK) return res;
    *tstep = v & 0xFFFFF;
    return TMC2209_OK;
}

/* ==== Utility ==== */

tmc2209_result_t tmc2209_last_error(const tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    return drv->last_error;
}

const char *tmc2209_result_str(tmc2209_result_t res)
{
    switch (res) {
        case TMC2209_OK:              return "OK";
        case TMC2209_ERR_TIMEOUT:     return "TIMEOUT";
        case TMC2209_ERR_UART:        return "UART_ERROR";
        case TMC2209_ERR_BAD_FRAME:   return "BAD_FRAME";
        case TMC2209_ERR_CRC:         return "CRC_ERROR";
        case TMC2209_ERR_INVALID_ARG: return "INVALID_ARG";
        case TMC2209_ERR_NOT_INIT:    return "NOT_INITIALIZED";
        case TMC2209_ERR_UNSUPPORTED: return "UNSUPPORTED";
        case TMC2209_ERR_HW:          return "HW_ERROR";
        default:                      return "UNKNOWN";
    }
}
