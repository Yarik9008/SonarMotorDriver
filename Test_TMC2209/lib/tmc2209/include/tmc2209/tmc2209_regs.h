/* tmc2209_regs.h — TMC2209 register addresses and bitfield definitions.
 *
 * Complete coverage of all TMC2209 registers per datasheet rev 1.09.
 * Access types: R = read-only, W = write-only, R/W, R/C = read/clear-on-write.
 */

#ifndef TMC2209_REGS_H
#define TMC2209_REGS_H

#include <stdint.h>

/* ==== Register addresses ==== */

#define TMC2209_REG_GCONF        0x00U  /* R/W */
#define TMC2209_REG_GSTAT        0x01U  /* R/C */
#define TMC2209_REG_IFCNT        0x02U  /* R   */
#define TMC2209_REG_SLAVECONF    0x03U  /* W   */
#define TMC2209_REG_OTP_PROG     0x04U  /* W   */
#define TMC2209_REG_OTP_READ     0x05U  /* R   */
#define TMC2209_REG_IOIN         0x06U  /* R   */
#define TMC2209_REG_FACTORY_CONF 0x07U  /* R/W */

#define TMC2209_REG_IHOLD_IRUN   0x10U  /* W   */
#define TMC2209_REG_TPOWERDOWN   0x11U  /* W   */
#define TMC2209_REG_TSTEP        0x12U  /* R   */
#define TMC2209_REG_TPWMTHRS     0x13U  /* W   */
#define TMC2209_REG_TCOOLTHRS    0x14U  /* W   */

#define TMC2209_REG_VACTUAL      0x22U  /* W   */

#define TMC2209_REG_SGTHRS       0x40U  /* W   */
#define TMC2209_REG_SG_RESULT    0x41U  /* R   */
#define TMC2209_REG_COOLCONF     0x42U  /* W   */

#define TMC2209_REG_MSCNT        0x6AU  /* R   */
#define TMC2209_REG_MSCURACT     0x6BU  /* R   */
#define TMC2209_REG_CHOPCONF     0x6CU  /* R/W */
#define TMC2209_REG_DRV_STATUS   0x6FU  /* R   */
#define TMC2209_REG_PWMCONF      0x70U  /* W   */
#define TMC2209_REG_PWM_SCALE    0x71U  /* R   */
#define TMC2209_REG_PWM_AUTO     0x72U  /* R   */

/* ==== Protocol constants ==== */

#define TMC2209_SYNC_BYTE          0x05U
#define TMC2209_MASTER_ADDR        0xFFU
#define TMC2209_WRITE_BIT          0x80U
#define TMC2209_VERSION_EXPECTED   0x21U

/* ==== GCONF (0x00) R/W ==== */

#define TMC2209_GCONF_I_SCALE_ANALOG_Pos    0
#define TMC2209_GCONF_INTERNAL_RSENSE_Pos   1
#define TMC2209_GCONF_EN_SPREADCYCLE_Pos    2
#define TMC2209_GCONF_SHAFT_Pos             3
#define TMC2209_GCONF_INDEX_OTPW_Pos        4
#define TMC2209_GCONF_INDEX_STEP_Pos        5
#define TMC2209_GCONF_PDN_DISABLE_Pos       6
#define TMC2209_GCONF_MSTEP_REG_SELECT_Pos  7
#define TMC2209_GCONF_MULTISTEP_FILT_Pos    8

#define TMC2209_GCONF_I_SCALE_ANALOG    (1U << 0)
#define TMC2209_GCONF_INTERNAL_RSENSE   (1U << 1)
#define TMC2209_GCONF_EN_SPREADCYCLE    (1U << 2)
#define TMC2209_GCONF_SHAFT             (1U << 3)
#define TMC2209_GCONF_INDEX_OTPW        (1U << 4)
#define TMC2209_GCONF_INDEX_STEP        (1U << 5)
#define TMC2209_GCONF_PDN_DISABLE       (1U << 6)
#define TMC2209_GCONF_MSTEP_REG_SELECT  (1U << 7)
#define TMC2209_GCONF_MULTISTEP_FILT    (1U << 8)

/* ==== GSTAT (0x01) R/C ==== */

#define TMC2209_GSTAT_RESET     (1U << 0)
#define TMC2209_GSTAT_DRV_ERR   (1U << 1)
#define TMC2209_GSTAT_UV_CP     (1U << 2)

/* ==== SLAVECONF (0x03) W ==== */

#define TMC2209_SENDDELAY_Pos   8
#define TMC2209_SENDDELAY_Msk   (0x0FUL << 8)
#define TMC2209_SENDDELAY(n)    ((uint32_t)((n) & 0x0FU) << 8)

/* ==== OTP_PROG (0x04) W ==== */

#define TMC2209_OTP_PROG_OTPBIT_Pos    0
#define TMC2209_OTP_PROG_OTPBIT_Msk    (0x07U << 0)
#define TMC2209_OTP_PROG_OTPBYTE_Pos   4
#define TMC2209_OTP_PROG_OTPBYTE_Msk   (0x03U << 4)
#define TMC2209_OTP_PROG_OTPMAGIC      (1U << 8)

#define TMC2209_OTP_PROG(byte, bit) \
    (TMC2209_OTP_PROG_OTPMAGIC | \
     ((uint32_t)((byte) & 0x03U) << 4) | \
     ((uint32_t)((bit) & 0x07U) << 0))

/* ==== OTP_READ (0x05) R ==== */

#define TMC2209_OTP_READ_BYTE0_Pos  0
#define TMC2209_OTP_READ_BYTE0_Msk  (0xFFUL << 0)
#define TMC2209_OTP_READ_BYTE1_Pos  8
#define TMC2209_OTP_READ_BYTE1_Msk  (0xFFUL << 8)
#define TMC2209_OTP_READ_BYTE2_Pos  16
#define TMC2209_OTP_READ_BYTE2_Msk  (0xFFUL << 16)

/* ==== IOIN (0x06) R ==== */

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

/* ==== FACTORY_CONF (0x07) R/W ==== */

#define TMC2209_FCLKTRIM_Pos  0
#define TMC2209_FCLKTRIM_Msk  (0x1FUL << 0)
#define TMC2209_OTTRIM_Pos    8
#define TMC2209_OTTRIM_Msk    (0x03UL << 8)

/* ==== IHOLD_IRUN (0x10) W ==== */

#define TMC2209_IHOLD_Pos      0
#define TMC2209_IHOLD_Msk      (0x1FUL << 0)
#define TMC2209_IRUN_Pos       8
#define TMC2209_IRUN_Msk       (0x1FUL << 8)
#define TMC2209_IHOLDDELAY_Pos 16
#define TMC2209_IHOLDDELAY_Msk (0x0FUL << 16)

#define TMC2209_IHOLD(n)      ((uint32_t)((n) & 0x1FU) << 0)
#define TMC2209_IRUN(n)       ((uint32_t)((n) & 0x1FU) << 8)
#define TMC2209_IHOLDDELAY(n) ((uint32_t)((n) & 0x0FU) << 16)

/* ==== CHOPCONF (0x6C) R/W ==== */

#define TMC2209_CHOPCONF_TOFF_Pos     0
#define TMC2209_CHOPCONF_TOFF_Msk     (0x0FUL << 0)
#define TMC2209_CHOPCONF_HSTRT_Pos    4
#define TMC2209_CHOPCONF_HSTRT_Msk    (0x07UL << 4)
#define TMC2209_CHOPCONF_HEND_Pos     7
#define TMC2209_CHOPCONF_HEND_Msk     (0x0FUL << 7)
#define TMC2209_CHOPCONF_TBL_Pos      15
#define TMC2209_CHOPCONF_TBL_Msk      (0x03UL << 15)
#define TMC2209_CHOPCONF_VSENSE_Pos   17
#define TMC2209_CHOPCONF_VSENSE       (1UL << 17)
#define TMC2209_CHOPCONF_MRES_Pos     24
#define TMC2209_CHOPCONF_MRES_Msk     (0x0FUL << 24)
#define TMC2209_CHOPCONF_MRES(n)      ((uint32_t)((n) & 0x0FU) << 24)
#define TMC2209_CHOPCONF_INTPOL       (1UL << 28)
#define TMC2209_CHOPCONF_DEDGE        (1UL << 29)
#define TMC2209_CHOPCONF_DISS2G       (1UL << 30)
#define TMC2209_CHOPCONF_DISS2VS      (1UL << 31)
#define TMC2209_CHOPCONF_DEFAULT      0x10000053U

/* ==== DRV_STATUS (0x6F) R ==== */

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

/* ==== PWMCONF (0x70) W ==== */

#define TMC2209_PWMCONF_OFS_Pos         0
#define TMC2209_PWMCONF_OFS_Msk         (0xFFUL << 0)
#define TMC2209_PWMCONF_GRAD_Pos        8
#define TMC2209_PWMCONF_GRAD_Msk        (0xFFUL << 8)
#define TMC2209_PWMCONF_FREQ_Pos        16
#define TMC2209_PWMCONF_FREQ_Msk        (0x03UL << 16)
#define TMC2209_PWMCONF_AUTOSCALE_Pos   18
#define TMC2209_PWMCONF_AUTOSCALE       (1UL << 18)
#define TMC2209_PWMCONF_AUTOGRAD_Pos    19
#define TMC2209_PWMCONF_AUTOGRAD        (1UL << 19)
#define TMC2209_PWMCONF_FREEWHEEL_Pos   20
#define TMC2209_PWMCONF_FREEWHEEL_Msk   (0x03UL << 20)
#define TMC2209_PWMCONF_REG_Pos         24
#define TMC2209_PWMCONF_REG_Msk         (0x0FUL << 24)
#define TMC2209_PWMCONF_LIM_Pos         28
#define TMC2209_PWMCONF_LIM_Msk         (0x0FUL << 28)

/* ==== COOLCONF (0x42) W ==== */

#define TMC2209_COOLCONF_SEMIN_Pos   0
#define TMC2209_COOLCONF_SEMIN_Msk   (0x0FUL << 0)
#define TMC2209_COOLCONF_SEUP_Pos    5
#define TMC2209_COOLCONF_SEUP_Msk    (0x03UL << 5)
#define TMC2209_COOLCONF_SEMAX_Pos   8
#define TMC2209_COOLCONF_SEMAX_Msk   (0x0FUL << 8)
#define TMC2209_COOLCONF_SEDN_Pos    13
#define TMC2209_COOLCONF_SEDN_Msk    (0x03UL << 13)
#define TMC2209_COOLCONF_SEIMIN_Pos  15
#define TMC2209_COOLCONF_SEIMIN      (1UL << 15)

/* ==== PWM_SCALE (0x71) R ==== */

#define TMC2209_PWM_SCALE_SUM_Pos    0
#define TMC2209_PWM_SCALE_SUM_Msk    (0xFFUL << 0)
#define TMC2209_PWM_SCALE_AUTO_Pos   16
#define TMC2209_PWM_SCALE_AUTO_Msk   (0x1FFUL << 16)

/* ==== PWM_AUTO (0x72) R ==== */

#define TMC2209_PWM_AUTO_OFS_Pos     0
#define TMC2209_PWM_AUTO_OFS_Msk     (0xFFUL << 0)
#define TMC2209_PWM_AUTO_GRAD_Pos    16
#define TMC2209_PWM_AUTO_GRAD_Msk    (0xFFUL << 16)

/* ==== MSCURACT (0x6B) R ==== */

#define TMC2209_MSCURACT_CUR_A_Pos   0
#define TMC2209_MSCURACT_CUR_A_Msk   (0x1FFUL << 0)
#define TMC2209_MSCURACT_CUR_B_Pos   16
#define TMC2209_MSCURACT_CUR_B_Msk   (0x1FFUL << 16)

#endif /* TMC2209_REGS_H */
