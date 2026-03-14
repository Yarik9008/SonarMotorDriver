/* Example 6: OTP (One-Time Programmable) memory access.
 *
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !! WARNING: OTP programming is IRREVERSIBLE!           !!
 * !! Each bit can only be changed from 0 to 1, NEVER    !!
 * !! back. The OTP can only be programmed a limited      !!
 * !! number of times. Use with extreme caution.          !!
 * !! Most applications do NOT need OTP programming.      !!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * OTP stores power-on defaults for:
 *   Byte 0: FCLKTRIM, S2_level, BBM, TBL
 *   Byte 1: IHOLDDELAY, IHOLD[3:0], en_SpreadCycle
 *   Byte 2: TPWMTHRS, PWM_GRAD, PWM_AUTOGRAD, PWM_OFS, PWM_REG
 *
 * Refer to TMC2209 datasheet section "OTP Memory" for exact bit layouts.
 */

#include "tmc2209/tmc2209.h"
#include <stdio.h>

void example_otp_read(tmc2209_t *drv)
{
    /* Safe: reading OTP has no side effects */
    tmc2209_otp_t otp;
    if (tmc2209_otp_read(drv, &otp) == TMC2209_OK) {
        printf("OTP byte0=0x%02X byte1=0x%02X byte2=0x%02X\n",
               otp.byte0, otp.byte1, otp.byte2);

        /* Decode FCLKTRIM from byte0 bits [4:0] */
        uint8_t fclktrim = otp.byte0 & 0x1F;
        printf("  FCLKTRIM (OTP) = %u\n", fclktrim);
    }
}

void example_otp_program(tmc2209_t *drv)
{
    /*
     * DANGER: This programs a single OTP bit.
     * The function reads before/after to verify.
     *
     * Example: set bit 5 of OTP byte 1 (en_SpreadCycle default).
     * ONLY do this if you are absolutely sure.
     */
    printf("Programming OTP byte 1, bit 5 (en_SpreadCycle)...\n");

    tmc2209_result_t res = tmc2209_otp_program_bit(drv, 1, 5);
    if (res == TMC2209_OK)
        printf("OTP bit programmed successfully\n");
    else
        printf("OTP programming FAILED: %s\n", tmc2209_result_str(res));
}
