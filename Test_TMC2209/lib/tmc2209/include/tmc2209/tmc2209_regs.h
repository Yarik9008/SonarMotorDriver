/* tmc2209_regs.h — TMC2209 register addresses and bitfield definitions. */

#ifndef TMC2209_REGS_H
#define TMC2209_REGS_H

#include <stdint.h>

/* ---- Register addresses ---- */

#define TMC2209_REG_GCONF        0x00U
#define TMC2209_REG_GSTAT        0x01U
#define TMC2209_REG_IFCNT        0x02U
#define TMC2209_REG_SLAVECONF    0x03U
#define TMC2209_REG_OTP_PROG     0x04U
#define TMC2209_REG_OTP_READ     0x05U
#define TMC2209_REG_IOIN         0x06U
#define TMC2209_REG_FACTORY_CONF 0x07U

#define TMC2209_REG_IHOLD_IRUN   0x10U
#define TMC2209_REG_TPOWERDOWN   0x11U
#define TMC2209_REG_TSTEP        0x12U
#define TMC2209_REG_TPWMTHRS     0x13U
#define TMC2209_REG_TCOOLTHRS    0x14U

#define TMC2209_REG_VACTUAL      0x22U

#define TMC2209_REG_SGTHRS       0x40U
#define TMC2209_REG_SG_RESULT    0x41U
#define TMC2209_REG_COOLCONF     0x42U

#define TMC2209_REG_MSCNT        0x6AU
#define TMC2209_REG_MSCURACT     0x6BU
#define TMC2209_REG_CHOPCONF     0x6CU
#define TMC2209_REG_DRV_STATUS   0x6FU
#define TMC2209_REG_PWMCONF      0x70U
#define TMC2209_REG_PWM_SCALE    0x71U
#define TMC2209_REG_PWM_AUTO     0x72U

/* ---- GCONF (0x00) bitfields ---- */

#define TMC2209_GCONF_I_SCALE_ANALOG    (1U << 0)
#define TMC2209_GCONF_INTERNAL_RSENSE   (1U << 1)
#define TMC2209_GCONF_EN_SPREADCYCLE    (1U << 2)
#define TMC2209_GCONF_SHAFT             (1U << 3)
#define TMC2209_GCONF_INDEX_OTPW        (1U << 4)
#define TMC2209_GCONF_INDEX_STEP        (1U << 5)
#define TMC2209_GCONF_PDN_DISABLE       (1U << 6)
#define TMC2209_GCONF_MSTEP_REG_SELECT  (1U << 7)
#define TMC2209_GCONF_MULTISTEP_FILT    (1U << 8)

/* ---- GSTAT (0x01) bitfields ---- */

#define TMC2209_GSTAT_RESET     (1U << 0)
#define TMC2209_GSTAT_DRV_ERR   (1U << 1)
#define TMC2209_GSTAT_UV_CP     (1U << 2)

/* ---- IHOLD_IRUN (0x10) field constructors ---- */

#define TMC2209_IHOLD(n)      ((uint32_t)((n) & 0x1FU) << 0)
#define TMC2209_IRUN(n)       ((uint32_t)((n) & 0x1FU) << 8)
#define TMC2209_IHOLDDELAY(n) ((uint32_t)((n) & 0x0FU) << 16)

/* ---- SLAVECONF (0x03) ---- */

#define TMC2209_SENDDELAY(n)  ((uint32_t)((n) & 0x0FU) << 8)

/* ---- CHOPCONF (0x6C) bitfields ---- */

#define TMC2209_CHOPCONF_TOFF_Pos       0
#define TMC2209_CHOPCONF_HSTRT_Pos      4
#define TMC2209_CHOPCONF_HEND_Pos       7
#define TMC2209_CHOPCONF_TBL_Pos        15
#define TMC2209_CHOPCONF_VSENSE_Pos     17
#define TMC2209_CHOPCONF_MRES_Pos       24
#define TMC2209_CHOPCONF_MRES_Msk       (0x0FUL << 24)
#define TMC2209_CHOPCONF_MRES(n)        ((uint32_t)((n) & 0x0FU) << 24)
#define TMC2209_CHOPCONF_INTPOL         (1UL << 28)
#define TMC2209_CHOPCONF_DEDGE          (1UL << 29)
#define TMC2209_CHOPCONF_DISS2G         (1UL << 30)
#define TMC2209_CHOPCONF_DISS2VS        (1UL << 31)
#define TMC2209_CHOPCONF_DEFAULT        0x10000053U

/* ---- DRV_STATUS (0x6F) bit positions ---- */

#define TMC2209_DRV_OTPW_Pos       0
#define TMC2209_DRV_OT_Pos         1
#define TMC2209_DRV_S2GA_Pos       2
#define TMC2209_DRV_S2GB_Pos       3
#define TMC2209_DRV_S2VSA_Pos      4
#define TMC2209_DRV_S2VSB_Pos      5
#define TMC2209_DRV_OLA_Pos        6
#define TMC2209_DRV_OLB_Pos        7
#define TMC2209_DRV_T120_Pos       8
#define TMC2209_DRV_T143_Pos       9
#define TMC2209_DRV_T150_Pos       10
#define TMC2209_DRV_T157_Pos       11
#define TMC2209_DRV_CS_ACTUAL_Pos  16
#define TMC2209_DRV_CS_ACTUAL_Msk  (0x1FUL << 16)
#define TMC2209_DRV_STEALTH_Pos    30
#define TMC2209_DRV_STST_Pos       31

/* ---- IOIN (0x06) bit positions ---- */

#define TMC2209_IOIN_ENN_Pos       0
#define TMC2209_IOIN_MS1_Pos       2
#define TMC2209_IOIN_MS2_Pos       3
#define TMC2209_IOIN_DIAG_Pos      4
#define TMC2209_IOIN_PDN_UART_Pos  6
#define TMC2209_IOIN_STEP_Pos      7
#define TMC2209_IOIN_SPREAD_EN_Pos 8
#define TMC2209_IOIN_DIR_Pos       9
#define TMC2209_IOIN_VERSION_Pos   24
#define TMC2209_IOIN_VERSION_Msk   (0xFFUL << 24)

/* ---- Protocol constants ---- */

#define TMC2209_VERSION_EXPECTED   0x21U
#define TMC2209_SYNC_BYTE          0x05U
#define TMC2209_MASTER_ADDR        0xFFU
#define TMC2209_WRITE_BIT          0x80U

#endif /* TMC2209_REGS_H */
