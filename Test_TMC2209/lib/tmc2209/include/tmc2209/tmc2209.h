/* tmc2209.h — публичный API библиотеки драйвера шагового двигателя TMC2209.
 *
 * Кроссплатформенный UART-драйвер для Trinamic TMC2209.
 * Доступ к регистрам и типизированный высокоуровневый API для StealthChop,
 * SpreadCycle, CoolStep, StallGuard, OTP, диагностики и мультиустройств по UART.
 *
 * Не потокобезопасен. Буфер отладочного вывода общий для всех экземпляров.
 */

#ifndef TMC2209_H
#define TMC2209_H

#include "tmc2209/tmc2209_types.h"
#include "tmc2209/tmc2209_regs.h"
#include "tmc2209/tmc2209_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==== Контекст драйвера ==== */

typedef struct {
    tmc2209_config_t cfg;
    tmc2209_io_t     io;
    tmc2209_result_t last_error;
    uint8_t          initialized;
    tmc2209_shadow_t shadow;
    struct {
        uint32_t ihold_irun_saved;
        uint32_t tpowerdown_saved;
        uint8_t  active;
    } standby;
} tmc2209_t;

/* ==== Инициализация и деинициализация ==== */

/** Блокирующая инициализация: настройка регистров из cfg и проверка связи. */
tmc2209_result_t tmc2209_init(tmc2209_t *drv, const tmc2209_config_t *cfg,
                              const tmc2209_io_t *io);
void             tmc2209_deinit(tmc2209_t *drv);

/* ==== Низкоуровневый доступ к регистрам ==== */

tmc2209_result_t tmc2209_read_reg(tmc2209_t *drv, uint8_t reg, uint32_t *value);
tmc2209_result_t tmc2209_write_reg(tmc2209_t *drv, uint8_t reg, uint32_t value);
tmc2209_result_t tmc2209_read_reg_addr(tmc2209_t *drv, uint8_t addr,
                                       uint8_t reg, uint32_t *value);

/* ==== GCONF ==== */

/** Чтение и декодирование GCONF из микросхемы. */
tmc2209_result_t tmc2209_get_gconf(tmc2209_t *drv, tmc2209_gconf_t *gc);
/** Инвертировать направление вала двигателя. */
tmc2209_result_t tmc2209_set_shaft(tmc2209_t *drv, uint8_t inverted);
/** Переключить в режим SpreadCycle. */
tmc2209_result_t tmc2209_enable_spreadcycle(tmc2209_t *drv);
/** Переключить в режим StealthChop. */
tmc2209_result_t tmc2209_enable_stealthchop(tmc2209_t *drv);
/**
 * Включить внутренние резисторы измерения тока.
 * ВНИМАНИЕ: меняет масштаб тока; применять до включения двигателя.
 */
tmc2209_result_t tmc2209_enable_internal_rsense(tmc2209_t *drv, uint8_t enable);

/* ==== Ток: IHOLD_IRUN ==== */

/** Установить ток движения и удержания в мА (пересчёт в CS по rsense). */
tmc2209_result_t tmc2209_set_current(tmc2209_t *drv, uint16_t run_ma, uint16_t hold_ma);
/** Установить только ток движения (ток удержания не меняется). */
tmc2209_result_t tmc2209_set_run_current(tmc2209_t *drv, uint16_t run_ma);
/** Установить только ток удержания (ток движения не меняется). */
tmc2209_result_t tmc2209_set_hold_current(tmc2209_t *drv, uint16_t hold_ma);
/** Получить текущие значения CS из теневого регистра. */
tmc2209_result_t tmc2209_get_current_config(tmc2209_t *drv, tmc2209_current_config_t *cc);
/** Установить IHOLDDELAY (0..15). */
tmc2209_result_t tmc2209_set_iholddelay(tmc2209_t *drv, uint8_t delay);
/** Записать регистр TPOWERDOWN (0..255). */
tmc2209_result_t tmc2209_set_tpowerdown(tmc2209_t *drv, uint8_t value);

/* ==== CHOPCONF ==== */

/** Записать CHOPCONF из типизированной структуры. Обновляет тень. */
tmc2209_result_t tmc2209_set_chopconf_config(tmc2209_t *drv, const tmc2209_chopconf_t *cc);
/** Прочитать CHOPCONF из микросхемы и декодировать. */
tmc2209_result_t tmc2209_get_chopconf_config(tmc2209_t *drv, tmc2209_chopconf_t *cc);
/** Записать CHOPCONF сырым 32-битным значением. */
tmc2209_result_t tmc2209_set_chopconf(tmc2209_t *drv, uint32_t value);
/** Установить разрешение микрошагов (1,2,4,8,16,32,64,128,256). */
tmc2209_result_t tmc2209_set_microsteps(tmc2209_t *drv, uint16_t ms);
/** Прочитать текущее разрешение микрошагов из микросхемы. */
tmc2209_result_t tmc2209_get_microsteps(tmc2209_t *drv, uint16_t *ms);
/** Включить/выключить интерполяцию до 256 микрошагов. */
tmc2209_result_t tmc2209_enable_interpolation(tmc2209_t *drv, uint8_t enable);
/** Включить/выключить двойной фронт STEP (и по нарастанию, и по спаду). */
tmc2209_result_t tmc2209_enable_double_edge_step(tmc2209_t *drv, uint8_t enable);

/* ==== PWMCONF ==== */

/** Записать PWMCONF из типизированной структуры (регистр только на запись, используется тень). */
tmc2209_result_t tmc2209_set_pwmconf_config(tmc2209_t *drv, const tmc2209_pwmconf_t *pc);
/** Получить текущий PWMCONF из тени (регистр только на запись). */
tmc2209_result_t tmc2209_get_pwmconf_config(tmc2209_t *drv, tmc2209_pwmconf_t *pc);
/** Записать PWMCONF сырым 32-битным значением. */
tmc2209_result_t tmc2209_set_pwmconf(tmc2209_t *drv, uint32_t value);
/** Установить режим холостого хода / стояния. */
tmc2209_result_t tmc2209_set_freewheel(tmc2209_t *drv, tmc2209_freewheel_t mode);

/* ==== CoolStep ==== */

/** Настроить CoolStep из типизированной структуры. Записывает тень COOLCONF. */
tmc2209_result_t tmc2209_set_coolstep_config(tmc2209_t *drv,
                                             const tmc2209_coolstep_config_t *cs);
/** Получить конфиг CoolStep из тени (COOLCONF только на запись). */
tmc2209_result_t tmc2209_get_coolstep_config(tmc2209_t *drv, tmc2209_coolstep_config_t *cs);
/** Установить порог скорости TCOOLTHRS для CoolStep/StallGuard. */
tmc2209_result_t tmc2209_set_tcoolthrs(tmc2209_t *drv, uint32_t threshold);

/* ==== StallGuard ==== */

/** Установить порог StallGuard (0..255). */
tmc2209_result_t tmc2209_set_sgthrs(tmc2209_t *drv, uint8_t threshold);
/** Получить порог StallGuard из тени (SGTHRS только на запись). */
tmc2209_result_t tmc2209_get_sgthrs(tmc2209_t *drv, uint8_t *threshold);
/** Удобная настройка: пороги StallGuard и TCOOLTHRS вместе. */
tmc2209_result_t tmc2209_configure_stallguard(tmc2209_t *drv,
                                              const tmc2209_stallguard_config_t *sg);

/* ==== Управление двигателем ==== */

tmc2209_result_t tmc2209_enable(tmc2209_t *drv);
tmc2209_result_t tmc2209_disable(tmc2209_t *drv);
tmc2209_result_t tmc2209_set_vactual(tmc2209_t *drv, int32_t velocity);
tmc2209_result_t tmc2209_stop(tmc2209_t *drv);

/* ==== Режим ожидания (standby) ==== */

/**
 * Войти в программный standby: выключить драйвер, обнулить ток удержания,
 * включить режим холостого хода. Предыдущие настройки сохраняются для восстановления.
 */
tmc2209_result_t tmc2209_enter_standby(tmc2209_t *drv);
/**
 * Выйти из standby: восстановить ток/задержку отключения и снова включить драйвер.
 */
tmc2209_result_t tmc2209_exit_standby(tmc2209_t *drv);

/* ==== Диагностика ==== */

tmc2209_result_t tmc2209_get_version(tmc2209_t *drv, uint8_t *version);
tmc2209_result_t tmc2209_get_ifcnt(tmc2209_t *drv, uint8_t *count);
tmc2209_result_t tmc2209_get_ioin(tmc2209_t *drv, tmc2209_ioin_t *ioin);
tmc2209_result_t tmc2209_get_drv_status(tmc2209_t *drv, tmc2209_drv_status_t *st);
tmc2209_result_t tmc2209_get_gstat(tmc2209_t *drv, tmc2209_gstat_t *gs);
/** Сбросить флаги GSTAT записью 1 в каждый бит. */
tmc2209_result_t tmc2209_clear_gstat(tmc2209_t *drv);
tmc2209_result_t tmc2209_get_sg_result(tmc2209_t *drv, uint16_t *result);
tmc2209_result_t tmc2209_get_tstep(tmc2209_t *drv, uint32_t *tstep);
/** Прочитать CS_ACTUAL из DRV_STATUS (0..31). */
tmc2209_result_t tmc2209_get_cs_actual(tmc2209_t *drv, uint8_t *cs);
tmc2209_result_t tmc2209_get_pwm_scale(tmc2209_t *drv, tmc2209_pwm_scale_t *ps);
tmc2209_result_t tmc2209_get_pwm_auto(tmc2209_t *drv, tmc2209_pwm_auto_t *pa);
tmc2209_result_t tmc2209_get_mscnt(tmc2209_t *drv, uint16_t *count);
tmc2209_result_t tmc2209_get_mscuract(tmc2209_t *drv, tmc2209_mscuract_t *mc);

/* ==== OTP ====
 *
 * ВНИМАНИЕ: биты OTP программируются только из 0 в 1.
 * Операция НЕОБРАТИМА и ограничена малым числом циклов записи.
 * Сначала прочитайте OTP; программируйте только при полном понимании последствий.
 */

/** Прочитать все 3 байта OTP. */
tmc2209_result_t tmc2209_otp_read(tmc2209_t *drv, tmc2209_otp_t *otp);
/**
 * Запрограммировать один бит OTP. byte_num=0..2, bit_num=0..7.
 * Чтение до/после для проверки. Возвращает ERR_HW при ошибке верификации.
 */
tmc2209_result_t tmc2209_otp_program_bit(tmc2209_t *drv,
                                         uint8_t byte_num, uint8_t bit_num);

/* ==== FACTORY_CONF ==== */

tmc2209_result_t tmc2209_get_factory_conf(tmc2209_t *drv, tmc2209_factory_conf_t *fc);
/** Установить подстройку внутренней частоты (0..31). Влияет на тайминг UART при внутреннем осцилляторе. */
tmc2209_result_t tmc2209_set_fclktrim(tmc2209_t *drv, uint8_t trim);

/* ==== Несколько устройств на шине ==== */

/**
 * Сканировать шину UART на наличие TMC2209 по адресам 0..3.
 * results — массив из 4 элементов tmc2209_scan_entry_t.
 * Возвращает количество найденных устройств.
 */
uint8_t tmc2209_scan_bus(tmc2209_t *drv, tmc2209_scan_entry_t results[4]);

/* ==== Предустановки ==== */

/** Применить настройки по умолчанию для StealthChop (GCONF + PWMCONF + CHOPCONF.toff). */
tmc2209_result_t tmc2209_apply_stealthchop_defaults(tmc2209_t *drv);
/** Применить настройки по умолчанию для SpreadCycle (GCONF + CHOPCONF). */
tmc2209_result_t tmc2209_apply_spreadcycle_defaults(tmc2209_t *drv);

/* ==== Служебные функции ==== */

tmc2209_result_t tmc2209_last_error(const tmc2209_t *drv);
const char      *tmc2209_result_str(tmc2209_result_t res);

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_H */
