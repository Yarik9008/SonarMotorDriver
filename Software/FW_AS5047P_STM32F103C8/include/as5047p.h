/**
 * @file    as5047p.h
 * @brief   Минимальный драйвер магнитного энкодера AMS AS5047P (SPI, 14 бит).
 *
 * Особенности протокола (datasheet AS5047P, раздел SPI Interface):
 *  - режим SPI 1: CPOL = 0, CPHA = 1, MSB first, кадр 16 бит, f_SCLK <= 10 МГц;
 *  - командный кадр: [15] чётность (even, по битам 14..0), [14] 1 = чтение /
 *    0 = запись, [13:0] адрес регистра;
 *  - ответный кадр: [15] чётность, [14] EF (ошибка команды), [13:0] данные;
 *  - чтение "с задержкой": данные регистра приходят в СЛЕДУЮЩЕМ кадре,
 *    поэтому одно чтение = два обмена (команда + NOP).
 */
#ifndef AS5047P_H
#define AS5047P_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* --- Адреса энергонезависимых (volatile) регистров --- */
#define AS5047P_REG_NOP        0x0000U
#define AS5047P_REG_ERRFL      0x0001U
#define AS5047P_REG_PROG       0x0003U
#define AS5047P_REG_DIAAGC     0x3FFCU
#define AS5047P_REG_MAG        0x3FFDU
#define AS5047P_REG_ANGLEUNC   0x3FFEU  /* угол без компенсации DAEC */
#define AS5047P_REG_ANGLECOM   0x3FFFU  /* угол с компенсацией DAEC */

/* --- Биты ERRFL (сбрасываются чтением регистра) --- */
#define AS5047P_ERRFL_FRERR    (1U << 0)  /* ошибка кадра (неверное число тактов) */
#define AS5047P_ERRFL_INVCOMM  (1U << 1)  /* несуществующая команда */
#define AS5047P_ERRFL_PARERR   (1U << 2)  /* ошибка чётности принятой команды */

/* --- Биты DIAAGC --- */
#define AS5047P_DIAAGC_AGC_Msk 0x00FFU    /* [7:0]  значение AGC: 0 = сильное поле */
#define AS5047P_DIAAGC_LF      (1U << 8)  /* смещение скомпенсировано (норма = 1) */
#define AS5047P_DIAAGC_COF     (1U << 9)  /* переполнение CORDIC, угол недостоверен */
#define AS5047P_DIAAGC_MAGH    (1U << 10) /* магнит слишком близко */
#define AS5047P_DIAAGC_MAGL    (1U << 11) /* магнит слишком далеко */

#define AS5047P_RESOLUTION     16384U     /* 2^14 отсчётов на оборот */

typedef enum {
    AS5047P_OK = 0,
    AS5047P_ERR_SPI,     /* ошибка/таймаут HAL_SPI_TransmitReceive */
    AS5047P_ERR_PARITY,  /* не сошлась чётность ответного кадра */
    AS5047P_ERR_FLAG     /* установлен бит EF — датчик не понял команду */
} as5047p_status_t;

typedef struct {
    SPI_HandleTypeDef *hspi;    /* уже настроенный SPI: 16 бит, CPOL 0, CPHA 1 */
    GPIO_TypeDef      *cs_port; /* порт линии CSn (управляется программно) */
    uint16_t           cs_pin;  /* вывод линии CSn */
} as5047p_t;

as5047p_status_t as5047p_read(as5047p_t *dev, uint16_t reg, uint16_t *value);
as5047p_status_t as5047p_write(as5047p_t *dev, uint16_t reg, uint16_t value);

/** Чтение ANGLECOM (0..16383). */
as5047p_status_t as5047p_read_angle(as5047p_t *dev, uint16_t *angle);

/** Перевод отсчётов в сотые доли градуса: 0..35999. */
uint32_t as5047p_raw_to_centideg(uint16_t raw);

/** Текстовое имя кода ошибки (для диагностической печати). */
const char *as5047p_strerror(as5047p_status_t st);

#endif /* AS5047P_H */
