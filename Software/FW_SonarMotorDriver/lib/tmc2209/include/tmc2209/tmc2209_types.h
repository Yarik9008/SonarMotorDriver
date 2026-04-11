/* tmc2209_types.h — TMC2209 library types: results, configs, decoded registers. */

#ifndef TMC2209_TYPES_H
#define TMC2209_TYPES_H

#include <stdint.h>

/* ==== Result codes ==== */

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

/* ==== Freewheel mode (PWMCONF bits [21:20]) ==== */

typedef enum {
    TMC2209_FREEWHEEL_NORMAL   = 0, /**< Normal operation */
    TMC2209_FREEWHEEL_ENABLED  = 1, /**< Freewheeling */
    TMC2209_FREEWHEEL_SHORT_LS = 2, /**< Coil shorted via low-side drivers */
    TMC2209_FREEWHEEL_SHORT_HS = 3, /**< Coil shorted via high-side drivers */
} tmc2209_freewheel_t;

/* ==== Driver init-time configuration ==== */

typedef struct {
    uint8_t  addr;              /**< Slave address 0..3 (MS1/MS2 pins) */
    float    rsense;            /**< Sense resistor, Ohm (e.g. 0.11) */
    uint16_t irun_ma;           /**< Run current, mA */
    uint16_t ihold_ma;          /**< Hold current, mA */
    uint16_t microsteps;        /**< 1,2,4,8,16,32,64,128,256 */
    uint16_t reply_delay_us;    /**< Delay after TX before RX, µs */
    uint8_t  iholddelay;        /**< IHOLDDELAY (0..15) */
    uint8_t  senddelay;         /**< SLAVECONF SENDDELAY (0..15) */
    uint32_t pwmconf;           /**< PWMCONF raw (0 = library default) */
    uint32_t tpwmthrs;          /**< TPWMTHRS velocity threshold */
    uint8_t  tpowerdown;        /**< TPOWERDOWN delay (0..255) */
    uint8_t  en_spreadcycle;    /**< 0=StealthChop (default), 1=SpreadCycle */
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

/* ==== Typed config structs for register groups ==== */

/** GCONF register decoded */
typedef struct {
    uint8_t i_scale_analog;    /**< 0=internal VREF, 1=external AIN */
    uint8_t internal_rsense;   /**< 0=external sense resistors, 1=internal */
    uint8_t en_spreadcycle;    /**< 0=StealthChop, 1=SpreadCycle */
    uint8_t shaft;             /**< 0=normal direction, 1=inverted */
    uint8_t index_otpw;        /**< INDEX: 0=microstep pos, 1=overtemp warning */
    uint8_t index_step;        /**< INDEX: 0=as configured, 1=step output */
    uint8_t pdn_disable;       /**< 0=PDN_UART controls standby, 1=UART active */
    uint8_t mstep_reg_select;  /**< 0=MS pins set µsteps, 1=MRES register */
    uint8_t multistep_filt;    /**< 0=no filter, 1=step pulse filter */
} tmc2209_gconf_t;

/** CHOPCONF register decoded */
typedef struct {
    uint8_t toff;    /**< Off-time 0..15 (0=driver disabled) */
    uint8_t hstrt;   /**< Hysteresis start 0..7 */
    uint8_t hend;    /**< Hysteresis end 0..15 (offset -3) */
    uint8_t tbl;     /**< Comparator blank time 0..3 */
    uint8_t vsense;  /**< 0=low sensitivity, 1=high sensitivity */
    uint8_t mres;    /**< Microstep resolution 0..8 (256→1) */
    uint8_t intpol;  /**< 0=off, 1=interpolate to 256 µsteps */
    uint8_t dedge;   /**< 0=rising STEP edge, 1=both edges */
    uint8_t diss2g;  /**< 1=disable short to GND protection */
    uint8_t diss2vs; /**< 1=disable short to VS protection */
} tmc2209_chopconf_t;

/** PWMCONF register decoded (write-only, read from shadow) */
typedef struct {
    uint8_t pwm_ofs;       /**< PWM offset 0..255 */
    uint8_t pwm_grad;      /**< PWM gradient 0..255 */
    uint8_t pwm_freq;      /**< PWM frequency 0..3 */
    uint8_t pwm_autoscale; /**< 0=off, 1=auto amplitude scaling */
    uint8_t pwm_autograd;  /**< 0=off, 1=auto gradient adaptation */
    uint8_t freewheel;     /**< Standstill mode, see tmc2209_freewheel_t */
    uint8_t pwm_reg;       /**< Regulation loop gradient 0..15 */
    uint8_t pwm_lim;       /**< PWM limit for switching 0..15 */
} tmc2209_pwmconf_t;

/** CoolStep configuration (COOLCONF register, write-only) */
typedef struct {
    uint8_t semin;  /**< Min StallGuard for CoolStep enable 0..15 (0=off) */
    uint8_t seup;   /**< Current increment size 0..3 (1/2/4/8) */
    uint8_t semax;  /**< StallGuard hysteresis 0..15 */
    uint8_t sedn;   /**< Current decrement speed 0..3 */
    uint8_t seimin; /**< 0=half CS, 1=quarter CS minimum current */
} tmc2209_coolstep_config_t;

/** StallGuard configuration */
typedef struct {
    uint8_t  sgthrs;     /**< StallGuard threshold 0..255 */
    uint32_t tcoolthrs;  /**< Velocity threshold for CoolStep/StallGuard */
} tmc2209_stallguard_config_t;

/** Current configuration (IHOLD_IRUN, decoded from shadow) */
typedef struct {
    uint8_t irun;       /**< CS value for run current (0..31) */
    uint8_t ihold;      /**< CS value for hold current (0..31) */
    uint8_t iholddelay; /**< Delay when switching to IHOLD (0..15) */
} tmc2209_current_config_t;

/** FACTORY_CONF register decoded */
typedef struct {
    uint8_t fclktrim; /**< Internal clock trim 0..31 */
    uint8_t ottrim;   /**< Overtemperature threshold trim 0..3 */
} tmc2209_factory_conf_t;

/* ==== Decoded diagnostic register structs ==== */

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
    uint8_t otpw;       /**< Over-temperature pre-warning */
    uint8_t ot;         /**< Over-temperature shutdown */
    uint8_t s2ga;       /**< Short to GND phase A */
    uint8_t s2gb;       /**< Short to GND phase B */
    uint8_t s2vsa;      /**< Short to VS phase A */
    uint8_t s2vsb;      /**< Short to VS phase B */
    uint8_t ola;        /**< Open load phase A */
    uint8_t olb;        /**< Open load phase B */
    uint8_t t120;       /**< Temperature >120°C */
    uint8_t t143;       /**< Temperature >143°C */
    uint8_t t150;       /**< Temperature >150°C */
    uint8_t t157;       /**< Temperature >157°C */
    uint8_t cs_actual;  /**< Actual motor current scale (0..31) */
    uint8_t stealth;    /**< StealthChop indicator */
    uint8_t stst;       /**< Standstill indicator */
} tmc2209_drv_status_t;

typedef struct {
    uint8_t  pwm_scale_sum;  /**< PWM actual scale [7:0] */
    int16_t  pwm_scale_auto; /**< PWM automatic scale, signed 9-bit */
} tmc2209_pwm_scale_t;

typedef struct {
    uint8_t pwm_ofs_auto;    /**< Auto PWM offset [7:0] */
    uint8_t pwm_grad_auto;   /**< Auto PWM gradient [7:0] */
} tmc2209_pwm_auto_t;

typedef struct {
    int16_t cur_a; /**< Phase A current, signed 9-bit */
    int16_t cur_b; /**< Phase B current, signed 9-bit */
} tmc2209_mscuract_t;

/* ==== Shadow registers (internal state for write-only register tracking) ==== */

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

/** OTP data (3 bytes read from OTP_READ) */
typedef struct {
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
} tmc2209_otp_t;

/** Bus scan result entry */
typedef struct {
    uint8_t  addr;
    uint8_t  found;     /**< 1 if TMC2209 responded at this address */
    uint8_t  version;   /**< IC version (expected 0x21) */
    uint32_t ioin_raw;  /**< Raw IOIN register value */
} tmc2209_scan_entry_t;

#endif /* TMC2209_TYPES_H */
