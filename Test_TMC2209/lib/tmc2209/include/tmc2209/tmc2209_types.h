/* tmc2209_types.h — типы библиотеки TMC2209: коды результата, конфиги, декодированные регистры. */

#ifndef TMC2209_TYPES_H
#define TMC2209_TYPES_H

#include <stdint.h>

/* ==== Коды результата ==== */

typedef enum {
    TMC2209_OK = 0,
    TMC2209_ERR_TIMEOUT,
    TMC2209_ERR_UART,
    TMC2209_ERR_BAD_FRAME,
    TMC2209_ERR_CRC,
    TMC2209_ERR_INVALID_ARG,
    TMC2209_ERR_NOT_INIT,
    TMC2209_ERR_UNSUPPORTED,
    TMC2209_ERR_HW,
} tmc2209_result_t;

/* ==== Режим холостого хода (биты PWMCONF [21:20]) ==== */

typedef enum {
    TMC2209_FREEWHEEL_NORMAL   = 0, /**< Нормальная работа */
    TMC2209_FREEWHEEL_ENABLED  = 1, /**< Холостой ход (freewheeling) */
    TMC2209_FREEWHEEL_SHORT_LS = 2, /**< Обмотка замкнута через нижние ключи */
    TMC2209_FREEWHEEL_SHORT_HS = 3, /**< Обмотка замкнута через верхние ключи */
} tmc2209_freewheel_t;

/* ==== Конфигурация при инициализации драйвера ==== */

typedef struct {
    uint8_t  addr;              /**< Адрес устройства 0..3 (выводы MS1/MS2) */
    float    rsense;            /**< Резистор измерения тока, Ом (напр. 0.11) */
    uint16_t irun_ma;           /**< Ток движения, мА */
    uint16_t ihold_ma;          /**< Ток удержания, мА */
    uint16_t microsteps;        /**< Микрошаги: 1,2,4,8,16,32,64,128,256 */
    uint16_t reply_delay_us;    /**< Задержка после TX перед RX, мкс */
    uint8_t  iholddelay;        /**< IHOLDDELAY (0..15) */
    uint8_t  senddelay;         /**< SLAVECONF SENDDELAY (0..15) */
    uint32_t pwmconf;           /**< Сырое значение PWMCONF (0 = по умолчанию в библиотеке) */
    uint32_t tpwmthrs;          /**< Порог скорости TPWMTHRS */
    uint8_t  tpowerdown;        /**< Задержка TPOWERDOWN (0..255) */
    uint8_t  en_spreadcycle;    /**< 0=StealthChop (по умолч.), 1=SpreadCycle */
} tmc2209_config_t;

#define TMC2209_DEFAULT_CONFIG { \
    .addr            = 0,        \
    .rsense          = 0.11f,    \
    .irun_ma         = 800,      \
    .ihold_ma        = 400,      \
    .microsteps      = 16,       \
    .reply_delay_us  = 500,      \
    .iholddelay      = 4,        \
    .senddelay       = 4,        \
    .pwmconf         = 0,        \
    .tpwmthrs        = 0,        \
    .tpowerdown      = 20,       \
    .en_spreadcycle  = 0,        \
}

/* ==== Типизированные структуры конфигурации по группам регистров ==== */

/** Декодированный регистр GCONF */
typedef struct {
    uint8_t i_scale_analog;    /**< 0=внутренний VREF, 1=внешний AIN */
    uint8_t internal_rsense;   /**< 0=внешние резисторы, 1=внутренние */
    uint8_t en_spreadcycle;    /**< 0=StealthChop, 1=SpreadCycle */
    uint8_t shaft;             /**< 0=нормальное направление, 1=инвертировано */
    uint8_t index_otpw;        /**< INDEX: 0=поз. микрошага, 1=предупр. перегрева */
    uint8_t index_step;        /**< INDEX: 0=как настроено, 1=выход шага */
    uint8_t pdn_disable;       /**< 0=PDN_UART управляет standby, 1=UART активен */
    uint8_t mstep_reg_select;  /**< 0=микрошаги с выводов MS, 1=из регистра MRES */
    uint8_t multistep_filt;    /**< 0=без фильтра, 1=фильтр импульсов STEP */
} tmc2209_gconf_t;

/** Декодированный регистр CHOPCONF */
typedef struct {
    uint8_t toff;    /**< Время выключения 0..15 (0=драйвер выключен) */
    uint8_t hstrt;   /**< Начало гистерезиса 0..7 */
    uint8_t hend;    /**< Конец гистерезиса 0..15 (смещение -3) */
    uint8_t tbl;     /**< Время blank компаратора 0..3 */
    uint8_t vsense;  /**< 0=низкая чувств., 1=высокая чувств. */
    uint8_t mres;    /**< Разрешение микрошагов 0..8 (256→1) */
    uint8_t intpol;  /**< 0=выкл., 1=интерполяция до 256 мкш */
    uint8_t dedge;   /**< 0=фронт STEP по нарастанию, 1=оба фронта */
    uint8_t diss2g;  /**< 1=отключить защиту от КЗ на GND */
    uint8_t diss2vs; /**< 1=отключить защиту от КЗ на VS */
} tmc2209_chopconf_t;

/** Декодированный регистр PWMCONF (только запись, чтение из тени) */
typedef struct {
    uint8_t pwm_ofs;       /**< Смещение ШИМ 0..255 */
    uint8_t pwm_grad;      /**< Градиент ШИМ 0..255 */
    uint8_t pwm_freq;      /**< Частота ШИМ 0..3 */
    uint8_t pwm_autoscale; /**< 0=выкл., 1=автомасштабирование амплитуды */
    uint8_t pwm_autograd;  /**< 0=выкл., 1=автоподстройка градиента */
    uint8_t freewheel;     /**< Режим стояния, см. tmc2209_freewheel_t */
    uint8_t pwm_reg;       /**< Градиент контура регулирования 0..15 */
    uint8_t pwm_lim;       /**< Лимит ШИМ для переключения 0..15 */
} tmc2209_pwmconf_t;

/** Конфигурация CoolStep (регистр COOLCONF, только запись) */
typedef struct {
    uint8_t semin;  /**< Мин. StallGuard для включения CoolStep 0..15 (0=выкл.) */
    uint8_t seup;   /**< Шаг увеличения тока 0..3 (1/2/4/8) */
    uint8_t semax;  /**< Гистерезис StallGuard 0..15 */
    uint8_t sedn;   /**< Скорость уменьшения тока 0..3 */
    uint8_t seimin; /**< 0=полов. CS, 1=четверть CS мин. ток */
} tmc2209_coolstep_config_t;

/** Конфигурация StallGuard */
typedef struct {
    uint8_t  sgthrs;     /**< Порог StallGuard 0..255 */
    uint32_t tcoolthrs;  /**< Порог скорости для CoolStep/StallGuard */
} tmc2209_stallguard_config_t;

/** Конфигурация тока (IHOLD_IRUN, декодировано из тени) */
typedef struct {
    uint8_t irun;       /**< Значение CS для тока движения (0..31) */
    uint8_t ihold;      /**< Значение CS для тока удержания (0..31) */
    uint8_t iholddelay; /**< Задержка при переходе в IHOLD (0..15) */
} tmc2209_current_config_t;

/** Декодированный регистр FACTORY_CONF */
typedef struct {
    uint8_t fclktrim; /**< Подстройка внутр. частоты 0..31 */
    uint8_t ottrim;   /**< Подстройка порога перегрева 0..3 */
} tmc2209_factory_conf_t;

/* ==== Декодированные структуры диагностических регистров ==== */

typedef struct {
    uint8_t reset;
    uint8_t drv_err;
    uint8_t uv_cp;
} tmc2209_gstat_t;

typedef struct {
    uint8_t enn;
    uint8_t ms1;
    uint8_t ms2;
    uint8_t diag;
    uint8_t pdn_uart;
    uint8_t step;
    uint8_t spread_en;
    uint8_t dir;
    uint8_t version;
} tmc2209_ioin_t;

typedef struct {
    uint8_t otpw;       /**< Предупреждение о перегреве */
    uint8_t ot;         /**< Отключение по перегреву */
    uint8_t s2ga;       /**< КЗ на GND фаза A */
    uint8_t s2gb;       /**< КЗ на GND фаза B */
    uint8_t s2vsa;      /**< КЗ на VS фаза A */
    uint8_t s2vsb;      /**< КЗ на VS фаза B */
    uint8_t ola;        /**< Обрыв обмотки фаза A */
    uint8_t olb;        /**< Обрыв обмотки фаза B */
    uint8_t t120;       /**< Температура >120°C */
    uint8_t t143;       /**< Температура >143°C */
    uint8_t t150;       /**< Температура >150°C */
    uint8_t t157;       /**< Температура >157°C */
    uint8_t cs_actual;  /**< Фактическая шкала тока двигателя (0..31) */
    uint8_t stealth;    /**< Признак режима StealthChop */
    uint8_t stst;       /**< Признак стояния */
} tmc2209_drv_status_t;

typedef struct {
    uint8_t  pwm_scale_sum;  /**< Фактическая шкала ШИМ [7:0] */
    int16_t  pwm_scale_auto; /**< Автошкала ШИМ, знаковое 9 бит */
} tmc2209_pwm_scale_t;

typedef struct {
    uint8_t pwm_ofs_auto;    /**< Автосмещение ШИМ [7:0] */
    uint8_t pwm_grad_auto;   /**< Автоградиент ШИМ [7:0] */
} tmc2209_pwm_auto_t;

typedef struct {
    int16_t cur_a; /**< Ток фазы A, знаковое 9 бит */
    int16_t cur_b; /**< Ток фазы B, знаковое 9 бит */
} tmc2209_mscuract_t;

/* ==== Теневые регистры (внутр. состояние для регистров только на запись) ==== */

typedef struct {
    uint32_t gconf;
    uint32_t slaveconf;
    uint32_t ihold_irun;
    uint32_t tpowerdown;
    uint32_t tpwmthrs;
    uint32_t tcoolthrs;
    uint32_t vactual;
    uint32_t sgthrs;
    uint32_t coolconf;
    uint32_t chopconf;
    uint32_t pwmconf;
    uint32_t factory_conf;
} tmc2209_shadow_t;

/** Данные OTP (3 байта из OTP_READ) */
typedef struct {
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
} tmc2209_otp_t;

/** Элемент результата сканирования шины */
typedef struct {
    uint8_t  addr;
    uint8_t  found;     /**< 1, если TMC2209 ответил по этому адресу */
    uint8_t  version;   /**< Версия ИС (ожидается 0x21) */
    uint32_t ioin_raw;  /**< Сырое значение регистра IOIN */
} tmc2209_scan_entry_t;

#endif /* TMC2209_TYPES_H */
