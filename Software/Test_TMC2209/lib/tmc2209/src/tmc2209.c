/* tmc2209.c — ядро библиотеки драйвера TMC2209.
 *
 * Кроссплатформенно: доступ к железу только через колбэки tmc2209_io_t.
 * Содержит UART-протокол, обмен с регистрами, тени регистров, инициализацию,
 * настройки, диагностику, OTP, CoolStep, StallGuard и предустановки.
 */

#include "tmc2209/tmc2209.h"
#include <string.h>
#include <stdio.h>

#define RX_BUF_SIZE 16U

/* ================================================================
 *  Внутренние вспомогательные функции
 * ================================================================ */

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

/* CRC8-ATM (полином 0x07, младший бит первым) */
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
        default:  return 8;  /* полный шаг по умолчанию; вызывающий должен проверять */
    }
}

static uint16_t mres_to_microsteps(uint8_t mres)
{
    if (mres > 8) return 256;
    return (uint16_t)(256U >> mres);
}

static uint8_t is_valid_microsteps(uint16_t ms)
{
    return ms > 0 && ms <= 256 && (ms & (ms - 1)) == 0;
}

/* ================================================================
 *  Низкоуровневый обмен с регистрами
 * ================================================================ */

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
    drv->last_error = TMC2209_OK;
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

    if (rx_ret < 0) {
        DBG(drv, "FAIL: UART error\r\n");
        drv->last_error = TMC2209_ERR_UART;
        return TMC2209_ERR_UART;
    }

    if (n == 0) {
        DBG(drv, "FAIL: no bytes\r\n");
        drv->last_error = TMC2209_ERR_TIMEOUT;
        return TMC2209_ERR_TIMEOUT;
    }

    uint8_t crc_fail = 0;
    /* Стандартный ответ 8 байт: [Sync][MasterAddr=0xFF][Reg][Val32][CRC] */
    for (uint8_t off = 0; off + 8 <= n; off++) {
        uint8_t *r = rx_buf + off;
        if (r[0] != TMC2209_SYNC_BYTE)   continue;
        if (r[1] != TMC2209_MASTER_ADDR) continue;
        if (r[2] != reg)                 continue;

        uint8_t crc_saved = r[7];
        r[7] = 0;
        tmc_crc(r, 8);
        if (crc_saved != r[7]) {
            DBG(drv, "FAIL: CRC mismatch @off=%u (got 0x%02X exp 0x%02X)\r\n",
                off, crc_saved, r[7]);
            r[7] = crc_saved;
            crc_fail = 1;
            continue;
        }

        *value = ((uint32_t)r[3] << 24) | ((uint32_t)r[4] << 16) |
                 ((uint32_t)r[5] << 8)  |  (uint32_t)r[6];
        drv->last_error = TMC2209_OK;
        return TMC2209_OK;
    }

    /* Вариант 7 байт: полный ответ 8 байт, один байт (0xFF) потерян при приёме — восстанавливаем и проверяем CRC */
    if (n >= 7) {
        for (uint8_t off = 0; off + 7 <= n; off++) {
            uint8_t *r = rx_buf + off;
            if (r[0] != TMC2209_SYNC_BYTE || r[1] != reg) continue;

            uint8_t full[8];
            full[0] = r[0];
            full[1] = TMC2209_MASTER_ADDR;
            full[2] = r[1];
            full[3] = r[2];
            full[4] = r[3];
            full[5] = r[4];
            full[6] = r[5];
            full[7] = 0;
            tmc_crc(full, 8);
            if (full[7] != r[6]) continue;

            *value = ((uint32_t)r[2] << 24) | ((uint32_t)r[3] << 16) |
                     ((uint32_t)r[4] << 8)  |  (uint32_t)r[5];
            drv->last_error = TMC2209_OK;
            return TMC2209_OK;
        }
    }

    if (crc_fail) {
        DBG(drv, "FAIL: frame found but CRC invalid\r\n");
        drv->last_error = TMC2209_ERR_CRC;
        return TMC2209_ERR_CRC;
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

/* ================================================================
 *  Работа с теневыми регистрами
 * ================================================================ */

static tmc2209_result_t shadow_write(tmc2209_t *drv, uint8_t reg,
                                     uint32_t *shadow, uint32_t value)
{
    tmc2209_result_t r = tmc2209_write_reg(drv, reg, value);
    if (r == TMC2209_OK)
        *shadow = value;
    return r;
}

/* Изменить отдельные биты в теневом регистре и записать. */
static tmc2209_result_t shadow_modify(tmc2209_t *drv, uint8_t reg,
                                      uint32_t *shadow, uint32_t mask, uint32_t bits)
{
    uint32_t val = (*shadow & ~mask) | (bits & mask);
    return shadow_write(drv, reg, shadow, val);
}

/* Собрать IHOLD_IRUN из отдельных полей */
static uint32_t build_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t iholddelay)
{
    return TMC2209_IHOLD(ihold) | TMC2209_IRUN(irun) | TMC2209_IHOLDDELAY(iholddelay);
}

/* ================================================================
 *  Инициализация и деинициализация
 * ================================================================ */

tmc2209_result_t tmc2209_init(tmc2209_t *drv, const tmc2209_config_t *cfg,
                              const tmc2209_io_t *io)
{
    if (!drv || !cfg || !io)          return TMC2209_ERR_INVALID_ARG;
    if (!io->uart_tx || !io->uart_rx) return TMC2209_ERR_INVALID_ARG;
    if (!io->uart_rx_flush)           return TMC2209_ERR_INVALID_ARG;
    if (!io->delay_us || !io->set_enable) return TMC2209_ERR_INVALID_ARG;

    if (cfg->addr > 3)                        return TMC2209_ERR_INVALID_ARG;
    if (cfg->rsense <= 0.0f)                   return TMC2209_ERR_INVALID_ARG;
    if (!is_valid_microsteps(cfg->microsteps)) return TMC2209_ERR_INVALID_ARG;
    if (cfg->iholddelay > 15)                  return TMC2209_ERR_INVALID_ARG;
    if (cfg->senddelay > 15)                   return TMC2209_ERR_INVALID_ARG;

    memset(drv, 0, sizeof(*drv));
    drv->cfg = *cfg;
    drv->io  = *io;

    drv->io.set_enable(1, drv->io.ctx);
    drv->io.delay_us(10000, drv->io.ctx);

    DBG(drv, "--- TMC2209 init ---\r\n");
    DBG(drv, "addr=%u, delay=%uus\r\n",
        (unsigned)cfg->addr, (unsigned)cfg->reply_delay_us);

    tmc2209_result_t res;

    /* GCONF */
    uint32_t gconf = TMC2209_GCONF_PDN_DISABLE |
                     TMC2209_GCONF_MSTEP_REG_SELECT |
                     TMC2209_GCONF_MULTISTEP_FILT;
    if (cfg->en_spreadcycle) gconf |= TMC2209_GCONF_EN_SPREADCYCLE;
    res = shadow_write(drv, TMC2209_REG_GCONF, &drv->shadow.gconf, gconf);
    if (res != TMC2209_OK) return res;

    /* IHOLD_IRUN */
    uint8_t irun  = current_to_cs(cfg->irun_ma, cfg->rsense);
    uint8_t ihold = current_to_cs(cfg->ihold_ma, cfg->rsense);
    if (ihold > irun) ihold = irun;
    uint32_t ihr = build_ihold_irun(ihold, irun, cfg->iholddelay);
    res = shadow_write(drv, TMC2209_REG_IHOLD_IRUN, &drv->shadow.ihold_irun, ihr);
    if (res != TMC2209_OK) return res;

    /* TPWMTHRS */
    res = shadow_write(drv, TMC2209_REG_TPWMTHRS, &drv->shadow.tpwmthrs, cfg->tpwmthrs);
    if (res != TMC2209_OK) return res;

    /* TPOWERDOWN */
    res = shadow_write(drv, TMC2209_REG_TPOWERDOWN, &drv->shadow.tpowerdown, cfg->tpowerdown);
    if (res != TMC2209_OK) return res;

    /* SLAVECONF */
    res = shadow_write(drv, TMC2209_REG_SLAVECONF, &drv->shadow.slaveconf,
                       TMC2209_SENDDELAY(cfg->senddelay));
    if (res != TMC2209_OK) return res;

    /* Проверка: IFCNT */
    uint32_t ifcnt = 0;
    res = tmc2209_read_reg(drv, TMC2209_REG_IFCNT, &ifcnt);
    if (res != TMC2209_OK) {
        DBG(drv, "init ERR: IFCNT read fail\r\n");
        return res;
    }
    DBG(drv, "init: IFCNT=%lu\r\n", (unsigned long)(ifcnt & 0xFF));

    /* Проверка: IOIN / VERSION */
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

    /* CHOPCONF: чтение-изменение-запись для MRES */
    uint32_t chop = 0;
    if (tmc2209_read_reg(drv, TMC2209_REG_CHOPCONF, &chop) != TMC2209_OK)
        chop = TMC2209_CHOPCONF_DEFAULT;
    chop = (chop & ~TMC2209_CHOPCONF_MRES_Msk) |
           TMC2209_CHOPCONF_MRES(microsteps_to_mres(cfg->microsteps));
    res = shadow_write(drv, TMC2209_REG_CHOPCONF, &drv->shadow.chopconf, chop);
    if (res != TMC2209_OK) return res;

    /* PWMCONF */
    uint32_t pwm = cfg->pwmconf ? cfg->pwmconf : 0xC10D0024U;
    res = shadow_write(drv, TMC2209_REG_PWMCONF, &drv->shadow.pwmconf, pwm);
    if (res != TMC2209_OK) return res;

    /* VACTUAL = 0 */
    res = shadow_write(drv, TMC2209_REG_VACTUAL, &drv->shadow.vactual, 0);
    if (res != TMC2209_OK) return res;

    /* Прочитать FACTORY_CONF в тень */
    (void)tmc2209_read_reg(drv, TMC2209_REG_FACTORY_CONF, &drv->shadow.factory_conf);

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

/* ================================================================
 *  GCONF
 * ================================================================ */

tmc2209_result_t tmc2209_get_gconf(tmc2209_t *drv, tmc2209_gconf_t *gc)
{
    if (!drv || !gc) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_GCONF, &v);
    if (r != TMC2209_OK) return r;
    drv->shadow.gconf = v;
    gc->i_scale_analog   = (v >> TMC2209_GCONF_I_SCALE_ANALOG_Pos) & 1;
    gc->internal_rsense  = (v >> TMC2209_GCONF_INTERNAL_RSENSE_Pos) & 1;
    gc->en_spreadcycle   = (v >> TMC2209_GCONF_EN_SPREADCYCLE_Pos) & 1;
    gc->shaft            = (v >> TMC2209_GCONF_SHAFT_Pos) & 1;
    gc->index_otpw       = (v >> TMC2209_GCONF_INDEX_OTPW_Pos) & 1;
    gc->index_step       = (v >> TMC2209_GCONF_INDEX_STEP_Pos) & 1;
    gc->pdn_disable      = (v >> TMC2209_GCONF_PDN_DISABLE_Pos) & 1;
    gc->mstep_reg_select = (v >> TMC2209_GCONF_MSTEP_REG_SELECT_Pos) & 1;
    gc->multistep_filt   = (v >> TMC2209_GCONF_MULTISTEP_FILT_Pos) & 1;
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_set_shaft(tmc2209_t *drv, uint8_t inverted)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_modify(drv, TMC2209_REG_GCONF, &drv->shadow.gconf,
                         TMC2209_GCONF_SHAFT,
                         inverted ? TMC2209_GCONF_SHAFT : 0);
}

tmc2209_result_t tmc2209_enable_spreadcycle(tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_modify(drv, TMC2209_REG_GCONF, &drv->shadow.gconf,
                         TMC2209_GCONF_EN_SPREADCYCLE,
                         TMC2209_GCONF_EN_SPREADCYCLE);
}

tmc2209_result_t tmc2209_enable_stealthchop(tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_modify(drv, TMC2209_REG_GCONF, &drv->shadow.gconf,
                         TMC2209_GCONF_EN_SPREADCYCLE, 0);
}

tmc2209_result_t tmc2209_enable_internal_rsense(tmc2209_t *drv, uint8_t enable)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_modify(drv, TMC2209_REG_GCONF, &drv->shadow.gconf,
                         TMC2209_GCONF_INTERNAL_RSENSE,
                         enable ? TMC2209_GCONF_INTERNAL_RSENSE : 0);
}

/* ================================================================
 *  Ток / IHOLD_IRUN
 * ================================================================ */

tmc2209_result_t tmc2209_set_current(tmc2209_t *drv, uint16_t run_ma, uint16_t hold_ma)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint8_t irun  = current_to_cs(run_ma, drv->cfg.rsense);
    uint8_t ihold = current_to_cs(hold_ma, drv->cfg.rsense);
    if (ihold > irun) ihold = irun;
    uint8_t ihd = (uint8_t)((drv->shadow.ihold_irun >> TMC2209_IHOLDDELAY_Pos) & 0x0F);
    uint32_t val = build_ihold_irun(ihold, irun, ihd);
    drv->cfg.irun_ma  = run_ma;
    drv->cfg.ihold_ma = hold_ma;
    return shadow_write(drv, TMC2209_REG_IHOLD_IRUN, &drv->shadow.ihold_irun, val);
}

tmc2209_result_t tmc2209_set_run_current(tmc2209_t *drv, uint16_t run_ma)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint8_t irun  = current_to_cs(run_ma, drv->cfg.rsense);
    uint8_t ihold = (uint8_t)((drv->shadow.ihold_irun >> TMC2209_IHOLD_Pos) & 0x1F);
    uint8_t ihd   = (uint8_t)((drv->shadow.ihold_irun >> TMC2209_IHOLDDELAY_Pos) & 0x0F);
    if (ihold > irun) ihold = irun;
    drv->cfg.irun_ma = run_ma;
    return shadow_write(drv, TMC2209_REG_IHOLD_IRUN, &drv->shadow.ihold_irun,
                        build_ihold_irun(ihold, irun, ihd));
}

tmc2209_result_t tmc2209_set_hold_current(tmc2209_t *drv, uint16_t hold_ma)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint8_t ihold = current_to_cs(hold_ma, drv->cfg.rsense);
    uint8_t irun  = (uint8_t)((drv->shadow.ihold_irun >> TMC2209_IRUN_Pos) & 0x1F);
    uint8_t ihd   = (uint8_t)((drv->shadow.ihold_irun >> TMC2209_IHOLDDELAY_Pos) & 0x0F);
    if (ihold > irun) ihold = irun;
    drv->cfg.ihold_ma = hold_ma;
    return shadow_write(drv, TMC2209_REG_IHOLD_IRUN, &drv->shadow.ihold_irun,
                        build_ihold_irun(ihold, irun, ihd));
}

tmc2209_result_t tmc2209_get_current_config(tmc2209_t *drv, tmc2209_current_config_t *cc)
{
    if (!drv || !cc) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t s = drv->shadow.ihold_irun;
    cc->ihold      = (uint8_t)((s >> TMC2209_IHOLD_Pos) & 0x1F);
    cc->irun       = (uint8_t)((s >> TMC2209_IRUN_Pos) & 0x1F);
    cc->iholddelay = (uint8_t)((s >> TMC2209_IHOLDDELAY_Pos) & 0x0F);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_set_iholddelay(tmc2209_t *drv, uint8_t delay)
{
    if (!drv || delay > 15) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_modify(drv, TMC2209_REG_IHOLD_IRUN, &drv->shadow.ihold_irun,
                         TMC2209_IHOLDDELAY_Msk, TMC2209_IHOLDDELAY(delay));
}

tmc2209_result_t tmc2209_set_tpowerdown(tmc2209_t *drv, uint8_t value)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_write(drv, TMC2209_REG_TPOWERDOWN, &drv->shadow.tpowerdown, value);
}

/* ================================================================
 *  CHOPCONF (чанпер)
 * ================================================================ */

static uint32_t chopconf_encode(const tmc2209_chopconf_t *cc)
{
    uint32_t v = 0;
    v |= ((uint32_t)(cc->toff   & 0x0F) << TMC2209_CHOPCONF_TOFF_Pos);
    v |= ((uint32_t)(cc->hstrt  & 0x07) << TMC2209_CHOPCONF_HSTRT_Pos);
    v |= ((uint32_t)(cc->hend   & 0x0F) << TMC2209_CHOPCONF_HEND_Pos);
    v |= ((uint32_t)(cc->tbl    & 0x03) << TMC2209_CHOPCONF_TBL_Pos);
    if (cc->vsense)  v |= TMC2209_CHOPCONF_VSENSE;
    v |= TMC2209_CHOPCONF_MRES(cc->mres & 0x0F);
    if (cc->intpol)  v |= TMC2209_CHOPCONF_INTPOL;
    if (cc->dedge)   v |= TMC2209_CHOPCONF_DEDGE;
    if (cc->diss2g)  v |= TMC2209_CHOPCONF_DISS2G;
    if (cc->diss2vs) v |= TMC2209_CHOPCONF_DISS2VS;
    return v;
}

static void chopconf_decode(uint32_t v, tmc2209_chopconf_t *cc)
{
    cc->toff    = (v >> TMC2209_CHOPCONF_TOFF_Pos)  & 0x0F;
    cc->hstrt   = (v >> TMC2209_CHOPCONF_HSTRT_Pos) & 0x07;
    cc->hend    = (v >> TMC2209_CHOPCONF_HEND_Pos)  & 0x0F;
    cc->tbl     = (v >> TMC2209_CHOPCONF_TBL_Pos)   & 0x03;
    cc->vsense  = (v >> TMC2209_CHOPCONF_VSENSE_Pos) & 1;
    cc->mres    = (v >> TMC2209_CHOPCONF_MRES_Pos)  & 0x0F;
    cc->intpol  = (v >> 28) & 1;
    cc->dedge   = (v >> 29) & 1;
    cc->diss2g  = (v >> 30) & 1;
    cc->diss2vs = (v >> 31) & 1;
}

tmc2209_result_t tmc2209_set_chopconf_config(tmc2209_t *drv, const tmc2209_chopconf_t *cc)
{
    if (!drv || !cc) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    if (cc->toff > 15 || cc->hstrt > 7 || cc->hend > 15 || cc->tbl > 3 || cc->mres > 8)
        return TMC2209_ERR_INVALID_ARG;
    return shadow_write(drv, TMC2209_REG_CHOPCONF, &drv->shadow.chopconf,
                        chopconf_encode(cc));
}

tmc2209_result_t tmc2209_get_chopconf_config(tmc2209_t *drv, tmc2209_chopconf_t *cc)
{
    if (!drv || !cc) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_CHOPCONF, &v);
    if (r != TMC2209_OK) return r;
    drv->shadow.chopconf = v;
    chopconf_decode(v, cc);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_set_chopconf(tmc2209_t *drv, uint32_t value)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_write(drv, TMC2209_REG_CHOPCONF, &drv->shadow.chopconf, value);
}

tmc2209_result_t tmc2209_set_microsteps(tmc2209_t *drv, uint16_t ms)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    if (!is_valid_microsteps(ms)) return TMC2209_ERR_INVALID_ARG;
    drv->cfg.microsteps = ms;
    return shadow_modify(drv, TMC2209_REG_CHOPCONF, &drv->shadow.chopconf,
                         TMC2209_CHOPCONF_MRES_Msk,
                         TMC2209_CHOPCONF_MRES(microsteps_to_mres(ms)));
}

tmc2209_result_t tmc2209_get_microsteps(tmc2209_t *drv, uint16_t *ms)
{
    if (!drv || !ms) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_CHOPCONF, &v);
    if (r != TMC2209_OK) return r;
    drv->shadow.chopconf = v;
    *ms = mres_to_microsteps((uint8_t)((v >> TMC2209_CHOPCONF_MRES_Pos) & 0x0F));
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_enable_interpolation(tmc2209_t *drv, uint8_t enable)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_modify(drv, TMC2209_REG_CHOPCONF, &drv->shadow.chopconf,
                         TMC2209_CHOPCONF_INTPOL,
                         enable ? TMC2209_CHOPCONF_INTPOL : 0);
}

tmc2209_result_t tmc2209_enable_double_edge_step(tmc2209_t *drv, uint8_t enable)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_modify(drv, TMC2209_REG_CHOPCONF, &drv->shadow.chopconf,
                         TMC2209_CHOPCONF_DEDGE,
                         enable ? TMC2209_CHOPCONF_DEDGE : 0);
}

/* ================================================================
 *  PWMCONF (ШИМ)
 * ================================================================ */

static uint32_t pwmconf_encode(const tmc2209_pwmconf_t *pc)
{
    uint32_t v = 0;
    v |= ((uint32_t)(pc->pwm_ofs       & 0xFF) << TMC2209_PWMCONF_OFS_Pos);
    v |= ((uint32_t)(pc->pwm_grad      & 0xFF) << TMC2209_PWMCONF_GRAD_Pos);
    v |= ((uint32_t)(pc->pwm_freq      & 0x03) << TMC2209_PWMCONF_FREQ_Pos);
    if (pc->pwm_autoscale) v |= TMC2209_PWMCONF_AUTOSCALE;
    if (pc->pwm_autograd)  v |= TMC2209_PWMCONF_AUTOGRAD;
    v |= ((uint32_t)(pc->freewheel     & 0x03) << TMC2209_PWMCONF_FREEWHEEL_Pos);
    v |= ((uint32_t)(pc->pwm_reg       & 0x0F) << TMC2209_PWMCONF_REG_Pos);
    v |= ((uint32_t)(pc->pwm_lim       & 0x0F) << TMC2209_PWMCONF_LIM_Pos);
    return v;
}

static void pwmconf_decode(uint32_t v, tmc2209_pwmconf_t *pc)
{
    pc->pwm_ofs       = (v >> TMC2209_PWMCONF_OFS_Pos)       & 0xFF;
    pc->pwm_grad      = (v >> TMC2209_PWMCONF_GRAD_Pos)      & 0xFF;
    pc->pwm_freq      = (v >> TMC2209_PWMCONF_FREQ_Pos)      & 0x03;
    pc->pwm_autoscale = (v >> TMC2209_PWMCONF_AUTOSCALE_Pos) & 1;
    pc->pwm_autograd  = (v >> TMC2209_PWMCONF_AUTOGRAD_Pos)  & 1;
    pc->freewheel     = (v >> TMC2209_PWMCONF_FREEWHEEL_Pos) & 0x03;
    pc->pwm_reg       = (v >> TMC2209_PWMCONF_REG_Pos)       & 0x0F;
    pc->pwm_lim       = (v >> TMC2209_PWMCONF_LIM_Pos)       & 0x0F;
}

tmc2209_result_t tmc2209_set_pwmconf_config(tmc2209_t *drv, const tmc2209_pwmconf_t *pc)
{
    if (!drv || !pc) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    if (pc->pwm_freq > 3 || pc->freewheel > 3 || pc->pwm_reg > 15 || pc->pwm_lim > 15)
        return TMC2209_ERR_INVALID_ARG;
    return shadow_write(drv, TMC2209_REG_PWMCONF, &drv->shadow.pwmconf,
                        pwmconf_encode(pc));
}

tmc2209_result_t tmc2209_get_pwmconf_config(tmc2209_t *drv, tmc2209_pwmconf_t *pc)
{
    if (!drv || !pc) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    pwmconf_decode(drv->shadow.pwmconf, pc);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_set_pwmconf(tmc2209_t *drv, uint32_t value)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_write(drv, TMC2209_REG_PWMCONF, &drv->shadow.pwmconf, value);
}

tmc2209_result_t tmc2209_set_freewheel(tmc2209_t *drv, tmc2209_freewheel_t mode)
{
    if (!drv || mode > 3) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_modify(drv, TMC2209_REG_PWMCONF, &drv->shadow.pwmconf,
                         TMC2209_PWMCONF_FREEWHEEL_Msk,
                         (uint32_t)mode << TMC2209_PWMCONF_FREEWHEEL_Pos);
}

/* ================================================================
 *  CoolStep
 * ================================================================ */

static uint32_t coolconf_encode(const tmc2209_coolstep_config_t *cs)
{
    uint32_t v = 0;
    v |= ((uint32_t)(cs->semin  & 0x0F) << TMC2209_COOLCONF_SEMIN_Pos);
    v |= ((uint32_t)(cs->seup   & 0x03) << TMC2209_COOLCONF_SEUP_Pos);
    v |= ((uint32_t)(cs->semax  & 0x0F) << TMC2209_COOLCONF_SEMAX_Pos);
    v |= ((uint32_t)(cs->sedn   & 0x03) << TMC2209_COOLCONF_SEDN_Pos);
    if (cs->seimin) v |= TMC2209_COOLCONF_SEIMIN;
    return v;
}

static void coolconf_decode(uint32_t v, tmc2209_coolstep_config_t *cs)
{
    cs->semin  = (v >> TMC2209_COOLCONF_SEMIN_Pos)  & 0x0F;
    cs->seup   = (v >> TMC2209_COOLCONF_SEUP_Pos)   & 0x03;
    cs->semax  = (v >> TMC2209_COOLCONF_SEMAX_Pos)  & 0x0F;
    cs->sedn   = (v >> TMC2209_COOLCONF_SEDN_Pos)   & 0x03;
    cs->seimin = (v >> TMC2209_COOLCONF_SEIMIN_Pos) & 1;
}

tmc2209_result_t tmc2209_set_coolstep_config(tmc2209_t *drv,
                                             const tmc2209_coolstep_config_t *cs)
{
    if (!drv || !cs) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    if (cs->semin > 15 || cs->seup > 3 || cs->semax > 15 || cs->sedn > 3)
        return TMC2209_ERR_INVALID_ARG;
    return shadow_write(drv, TMC2209_REG_COOLCONF, &drv->shadow.coolconf,
                        coolconf_encode(cs));
}

tmc2209_result_t tmc2209_get_coolstep_config(tmc2209_t *drv, tmc2209_coolstep_config_t *cs)
{
    if (!drv || !cs) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    coolconf_decode(drv->shadow.coolconf, cs);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_set_tcoolthrs(tmc2209_t *drv, uint32_t threshold)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_write(drv, TMC2209_REG_TCOOLTHRS, &drv->shadow.tcoolthrs, threshold);
}

/* ================================================================
 *  StallGuard
 * ================================================================ */

tmc2209_result_t tmc2209_set_sgthrs(tmc2209_t *drv, uint8_t threshold)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_write(drv, TMC2209_REG_SGTHRS, &drv->shadow.sgthrs, threshold);
}

tmc2209_result_t tmc2209_get_sgthrs(tmc2209_t *drv, uint8_t *threshold)
{
    if (!drv || !threshold) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    *threshold = (uint8_t)(drv->shadow.sgthrs & 0xFF);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_configure_stallguard(tmc2209_t *drv,
                                              const tmc2209_stallguard_config_t *sg)
{
    if (!drv || !sg) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    tmc2209_result_t r = tmc2209_set_sgthrs(drv, sg->sgthrs);
    if (r != TMC2209_OK) return r;
    return tmc2209_set_tcoolthrs(drv, sg->tcoolthrs);
}

/* ================================================================
 *  Управление двигателем
 * ================================================================ */

tmc2209_result_t tmc2209_enable(tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    drv->io.set_enable(0, drv->io.ctx);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_disable(tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    drv->io.set_enable(1, drv->io.ctx);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_set_vactual(tmc2209_t *drv, int32_t velocity)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_write(drv, TMC2209_REG_VACTUAL, &drv->shadow.vactual,
                        (uint32_t)velocity);
}

tmc2209_result_t tmc2209_stop(tmc2209_t *drv)
{
    return tmc2209_set_vactual(drv, 0);
}

/* ================================================================
 *  Режим ожидания (standby)
 * ================================================================ */

tmc2209_result_t tmc2209_enter_standby(tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    if (drv->standby.active) return TMC2209_OK;

    drv->standby.ihold_irun_saved = drv->shadow.ihold_irun;
    drv->standby.tpowerdown_saved = drv->shadow.tpowerdown;

    tmc2209_stop(drv);
    tmc2209_disable(drv);

    uint8_t irun = (uint8_t)((drv->shadow.ihold_irun >> TMC2209_IRUN_Pos) & 0x1F);
    shadow_write(drv, TMC2209_REG_IHOLD_IRUN, &drv->shadow.ihold_irun,
                 build_ihold_irun(0, irun, 0));
    shadow_write(drv, TMC2209_REG_TPOWERDOWN, &drv->shadow.tpowerdown, 0);
    tmc2209_set_freewheel(drv, TMC2209_FREEWHEEL_ENABLED);

    drv->standby.active = 1;
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_exit_standby(tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    if (!drv->standby.active) return TMC2209_OK;

    tmc2209_set_freewheel(drv, TMC2209_FREEWHEEL_NORMAL);
    shadow_write(drv, TMC2209_REG_IHOLD_IRUN, &drv->shadow.ihold_irun,
                 drv->standby.ihold_irun_saved);
    shadow_write(drv, TMC2209_REG_TPOWERDOWN, &drv->shadow.tpowerdown,
                 drv->standby.tpowerdown_saved);
    tmc2209_enable(drv);

    drv->standby.active = 0;
    return TMC2209_OK;
}

/* ================================================================
 *  Диагностика
 * ================================================================ */

tmc2209_result_t tmc2209_get_version(tmc2209_t *drv, uint8_t *version)
{
    if (!drv || !version) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_IOIN, &v);
    if (r != TMC2209_OK) return r;
    *version = (uint8_t)((v >> TMC2209_IOIN_VERSION_Pos) & 0xFF);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_ifcnt(tmc2209_t *drv, uint8_t *count)
{
    if (!drv || !count) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_IFCNT, &v);
    if (r != TMC2209_OK) return r;
    *count = (uint8_t)(v & 0xFF);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_ioin(tmc2209_t *drv, tmc2209_ioin_t *ioin)
{
    if (!drv || !ioin) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_IOIN, &v);
    if (r != TMC2209_OK) return r;
    ioin->enn       = (v >> TMC2209_IOIN_ENN_Pos)      & 1;
    ioin->ms1       = (v >> TMC2209_IOIN_MS1_Pos)      & 1;
    ioin->ms2       = (v >> TMC2209_IOIN_MS2_Pos)      & 1;
    ioin->diag      = (v >> TMC2209_IOIN_DIAG_Pos)     & 1;
    ioin->pdn_uart  = (v >> TMC2209_IOIN_PDN_UART_Pos) & 1;
    ioin->step      = (v >> TMC2209_IOIN_STEP_Pos)     & 1;
    ioin->spread_en = (v >> TMC2209_IOIN_SPREAD_EN_Pos) & 1;
    ioin->dir       = (v >> TMC2209_IOIN_DIR_Pos)      & 1;
    ioin->version   = (uint8_t)((v >> TMC2209_IOIN_VERSION_Pos) & 0xFF);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_drv_status(tmc2209_t *drv, tmc2209_drv_status_t *st)
{
    if (!drv || !st) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_DRV_STATUS, &v);
    if (r != TMC2209_OK) return r;
    st->otpw      = (v >> TMC2209_DRV_OTPW_Pos)  & 1;
    st->ot        = (v >> TMC2209_DRV_OT_Pos)    & 1;
    st->s2ga      = (v >> TMC2209_DRV_S2GA_Pos)  & 1;
    st->s2gb      = (v >> TMC2209_DRV_S2GB_Pos)  & 1;
    st->s2vsa     = (v >> TMC2209_DRV_S2VSA_Pos) & 1;
    st->s2vsb     = (v >> TMC2209_DRV_S2VSB_Pos) & 1;
    st->ola       = (v >> TMC2209_DRV_OLA_Pos)   & 1;
    st->olb       = (v >> TMC2209_DRV_OLB_Pos)   & 1;
    st->t120      = (v >> TMC2209_DRV_T120_Pos)  & 1;
    st->t143      = (v >> TMC2209_DRV_T143_Pos)  & 1;
    st->t150      = (v >> TMC2209_DRV_T150_Pos)  & 1;
    st->t157      = (v >> TMC2209_DRV_T157_Pos)  & 1;
    st->cs_actual = (uint8_t)((v & TMC2209_DRV_CS_ACTUAL_Msk) >> TMC2209_DRV_CS_ACTUAL_Pos);
    st->stealth   = (v >> TMC2209_DRV_STEALTH_Pos) & 1;
    st->stst      = (v >> TMC2209_DRV_STST_Pos)   & 1;
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_gstat(tmc2209_t *drv, tmc2209_gstat_t *gs)
{
    if (!drv || !gs) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_GSTAT, &v);
    if (r != TMC2209_OK) return r;
    gs->reset   = (v >> 0) & 1;
    gs->drv_err = (v >> 1) & 1;
    gs->uv_cp   = (v >> 2) & 1;
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_clear_gstat(tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return tmc2209_write_reg(drv, TMC2209_REG_GSTAT, 0x07);
}

tmc2209_result_t tmc2209_get_sg_result(tmc2209_t *drv, uint16_t *result)
{
    if (!drv || !result) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_SG_RESULT, &v);
    if (r != TMC2209_OK) return r;
    *result = (uint16_t)(v & 0x3FF);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_tstep(tmc2209_t *drv, uint32_t *tstep)
{
    if (!drv || !tstep) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_TSTEP, &v);
    if (r != TMC2209_OK) return r;
    *tstep = v & 0xFFFFF;
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_cs_actual(tmc2209_t *drv, uint8_t *cs)
{
    if (!drv || !cs) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_DRV_STATUS, &v);
    if (r != TMC2209_OK) return r;
    *cs = (uint8_t)((v & TMC2209_DRV_CS_ACTUAL_Msk) >> TMC2209_DRV_CS_ACTUAL_Pos);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_pwm_scale(tmc2209_t *drv, tmc2209_pwm_scale_t *ps)
{
    if (!drv || !ps) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_PWM_SCALE, &v);
    if (r != TMC2209_OK) return r;
    ps->pwm_scale_sum  = (uint8_t)(v & TMC2209_PWM_SCALE_SUM_Msk);
    int16_t raw = (int16_t)((v & TMC2209_PWM_SCALE_AUTO_Msk) >> TMC2209_PWM_SCALE_AUTO_Pos);
    if (raw & 0x100) raw -= 512;
    ps->pwm_scale_auto = raw;
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_pwm_auto(tmc2209_t *drv, tmc2209_pwm_auto_t *pa)
{
    if (!drv || !pa) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_PWM_AUTO, &v);
    if (r != TMC2209_OK) return r;
    pa->pwm_ofs_auto  = (uint8_t)(v & TMC2209_PWM_AUTO_OFS_Msk);
    pa->pwm_grad_auto = (uint8_t)((v & TMC2209_PWM_AUTO_GRAD_Msk) >> TMC2209_PWM_AUTO_GRAD_Pos);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_mscnt(tmc2209_t *drv, uint16_t *count)
{
    if (!drv || !count) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_MSCNT, &v);
    if (r != TMC2209_OK) return r;
    *count = (uint16_t)(v & 0x3FF);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_get_mscuract(tmc2209_t *drv, tmc2209_mscuract_t *mc)
{
    if (!drv || !mc) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_MSCURACT, &v);
    if (r != TMC2209_OK) return r;
    int16_t a = (int16_t)(v & TMC2209_MSCURACT_CUR_A_Msk);
    if (a & 0x100) a -= 512;
    mc->cur_a = a;
    int16_t b = (int16_t)((v & TMC2209_MSCURACT_CUR_B_Msk) >> TMC2209_MSCURACT_CUR_B_Pos);
    if (b & 0x100) b -= 512;
    mc->cur_b = b;
    return TMC2209_OK;
}

/* ================================================================
 *  OTP (однократно программируемая память)
 * ================================================================ */

tmc2209_result_t tmc2209_otp_read(tmc2209_t *drv, tmc2209_otp_t *otp)
{
    if (!drv || !otp) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_OTP_READ, &v);
    if (r != TMC2209_OK) return r;
    otp->byte0 = (uint8_t)((v >> TMC2209_OTP_READ_BYTE0_Pos) & 0xFF);
    otp->byte1 = (uint8_t)((v >> TMC2209_OTP_READ_BYTE1_Pos) & 0xFF);
    otp->byte2 = (uint8_t)((v >> TMC2209_OTP_READ_BYTE2_Pos) & 0xFF);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_otp_program_bit(tmc2209_t *drv,
                                         uint8_t byte_num, uint8_t bit_num)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    if (byte_num > 2 || bit_num > 7) return TMC2209_ERR_INVALID_ARG;

    /* Прочитать текущее состояние OTP */
    tmc2209_otp_t before;
    tmc2209_result_t r = tmc2209_otp_read(drv, &before);
    if (r != TMC2209_OK) return r;

    const uint8_t *bytes = &before.byte0;
    if (bytes[byte_num] & (1U << bit_num))
        return TMC2209_OK; /* бит уже установлен */

    /* Программирование: запись OTP_PROG с OTPMAGIC=1 */
    uint32_t prog = TMC2209_OTP_PROG(byte_num, bit_num);
    r = tmc2209_write_reg(drv, TMC2209_REG_OTP_PROG, prog);
    if (r != TMC2209_OK) return r;

    drv->io.delay_us(10000, drv->io.ctx);

    /* Проверка результата */
    tmc2209_otp_t after;
    r = tmc2209_otp_read(drv, &after);
    if (r != TMC2209_OK) return r;

    const uint8_t *abytes = &after.byte0;
    if (!(abytes[byte_num] & (1U << bit_num))) {
        drv->last_error = TMC2209_ERR_HW;
        return TMC2209_ERR_HW;
    }
    return TMC2209_OK;
}

/* ================================================================
 *  FACTORY_CONF (заводские настройки)
 * ================================================================ */

tmc2209_result_t tmc2209_get_factory_conf(tmc2209_t *drv, tmc2209_factory_conf_t *fc)
{
    if (!drv || !fc) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    uint32_t v = 0;
    tmc2209_result_t r = tmc2209_read_reg(drv, TMC2209_REG_FACTORY_CONF, &v);
    if (r != TMC2209_OK) return r;
    drv->shadow.factory_conf = v;
    fc->fclktrim = (uint8_t)((v & TMC2209_FCLKTRIM_Msk) >> TMC2209_FCLKTRIM_Pos);
    fc->ottrim   = (uint8_t)((v & TMC2209_OTTRIM_Msk) >> TMC2209_OTTRIM_Pos);
    return TMC2209_OK;
}

tmc2209_result_t tmc2209_set_fclktrim(tmc2209_t *drv, uint8_t trim)
{
    if (!drv || trim > 31) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    return shadow_modify(drv, TMC2209_REG_FACTORY_CONF, &drv->shadow.factory_conf,
                         TMC2209_FCLKTRIM_Msk,
                         (uint32_t)trim << TMC2209_FCLKTRIM_Pos);
}

/* ================================================================
 *  Несколько устройств на шине
 * ================================================================ */

uint8_t tmc2209_scan_bus(tmc2209_t *drv, tmc2209_scan_entry_t results[4])
{
    if (!drv || !results) return 0;
    uint8_t found = 0;
    for (uint8_t a = 0; a < 4; a++) {
        results[a].addr     = a;
        results[a].found    = 0;
        results[a].version  = 0;
        results[a].ioin_raw = 0;

        uint32_t v = 0;
        if (tmc2209_read_reg_addr(drv, a, TMC2209_REG_IOIN, &v) == TMC2209_OK) {
            uint8_t ver = (uint8_t)((v >> TMC2209_IOIN_VERSION_Pos) & 0xFF);
            results[a].ioin_raw = v;
            results[a].version  = ver;
            if (ver == TMC2209_VERSION_EXPECTED) {
                results[a].found = 1;
                found++;
            }
        }
    }
    return found;
}

/* ================================================================
 *  Предустановки
 * ================================================================ */

tmc2209_result_t tmc2209_apply_stealthchop_defaults(tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    tmc2209_result_t r;

    r = tmc2209_enable_stealthchop(drv);
    if (r != TMC2209_OK) return r;

    tmc2209_pwmconf_t pc = {
        .pwm_ofs       = 36,
        .pwm_grad      = 0,
        .pwm_freq      = 1,
        .pwm_autoscale = 1,
        .pwm_autograd  = 1,
        .freewheel     = 0,
        .pwm_reg       = 8,
        .pwm_lim       = 12,
    };
    r = tmc2209_set_pwmconf_config(drv, &pc);
    if (r != TMC2209_OK) return r;

    /* Убедиться, что чанпер включён (toff > 0) */
    return shadow_modify(drv, TMC2209_REG_CHOPCONF, &drv->shadow.chopconf,
                         TMC2209_CHOPCONF_TOFF_Msk,
                         3U << TMC2209_CHOPCONF_TOFF_Pos);
}

tmc2209_result_t tmc2209_apply_spreadcycle_defaults(tmc2209_t *drv)
{
    if (!drv) return TMC2209_ERR_INVALID_ARG;
    if (!drv->initialized) return TMC2209_ERR_NOT_INIT;
    tmc2209_result_t r;

    r = tmc2209_enable_spreadcycle(drv);
    if (r != TMC2209_OK) return r;

    tmc2209_chopconf_t cc;
    chopconf_decode(drv->shadow.chopconf, &cc);
    cc.toff  = 5;
    cc.hstrt = 4;
    cc.hend  = 1;
    cc.tbl   = 2;
    return tmc2209_set_chopconf_config(drv, &cc);
}

/* ================================================================
 *  Служебные функции
 * ================================================================ */

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
