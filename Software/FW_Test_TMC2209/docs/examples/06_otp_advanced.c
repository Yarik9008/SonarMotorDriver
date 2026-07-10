/* Пример 6: доступ к памяти OTP (однократно программируемой).
 *
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !! ВНИМАНИЕ: программирование OTP НЕОБРАТИМО!           !!
 * !! Каждый бит можно изменить только с 0 на 1, НИКОГДА !!
 * !! обратно. OTP можно запрограммировать ограниченное    !!
 * !! число раз. Использовать с крайней осторожностью.     !!
 * !! Большинству приложений OTP не нужен.                !!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * В OTP хранятся значения по умолчанию при включении:
 *   Байт 0: FCLKTRIM, S2_level, BBM, TBL
 *   Байт 1: IHOLDDELAY, IHOLD[3:0], en_SpreadCycle
 *   Байт 2: TPWMTHRS, PWM_GRAD, PWM_AUTOGRAD, PWM_OFS, PWM_REG
 *
 * Точная разводка бит — в разделе "OTP Memory" даташита TMC2209.
 */

#include "tmc2209/tmc2209.h"
#include <stdio.h>

void example_otp_read(tmc2209_t *drv)
{
    /* Безопасно: чтение OTP не имеет побочных эффектов */
    tmc2209_otp_t otp;
    if (tmc2209_otp_read(drv, &otp) == TMC2209_OK) {
        printf("OTP byte0=0x%02X byte1=0x%02X byte2=0x%02X\n",
               otp.byte0, otp.byte1, otp.byte2);

        /* Декодирование FCLKTRIM из бит [4:0] байта 0 */
        uint8_t fclktrim = otp.byte0 & 0x1F;
        printf("  FCLKTRIM (OTP) = %u\n", fclktrim);
    }
}

void example_otp_program(tmc2209_t *drv)
{
    /*
     * ОПАСНО: здесь программируется один бит OTP.
     * Функция читает до и после для проверки.
     *
     * Пример: установить бит 5 байта 1 OTP (en_SpreadCycle по умолчанию).
     * Делать только если вы полностью уверены.
     */
    printf("Программирование OTP байт 1, бит 5 (en_SpreadCycle)...\n");

    tmc2209_result_t res = tmc2209_otp_program_bit(drv, 1, 5);
    if (res == TMC2209_OK)
        printf("Бит OTP успешно запрограммирован\n");
    else
        printf("Ошибка программирования OTP: %s\n", tmc2209_result_str(res));
}
