#include "as5047p.h"

#define AS5047P_CMD_READ      0x4000U  /* бит 14 командного кадра: 1 = чтение */
#define AS5047P_FRAME_EF      0x4000U  /* бит 14 ответного кадра: флаг ошибки */
#define AS5047P_DATA_Msk      0x3FFFU
#define AS5047P_SPI_TIMEOUT   10U      /* мс, с запасом: кадр 16 бит идёт < 10 мкс */

/** Чётность (XOR всех битов слова). */
static uint16_t parity(uint16_t frame)
{
    frame ^= (uint16_t)(frame >> 8);
    frame ^= (uint16_t)(frame >> 4);
    frame ^= (uint16_t)(frame >> 2);
    frame ^= (uint16_t)(frame >> 1);
    return (uint16_t)(frame & 1U);
}

/** Дополнение кадра битом чётности (even parity по битам 14..0). */
static uint16_t with_parity(uint16_t frame)
{
    return (uint16_t)(frame | (uint16_t)(parity(frame) << 15));
}

/**
 * Пауза вокруг фронтов CSn: datasheet требует t_CSn >= 350 нс до первого такта
 * и между кадрами. На 72 МГц цикл volatile-счётчика ~5 тактов => ~0.7 мкс.
 */
static void delay_cs(void)
{
    for (volatile uint32_t i = 0; i < 10U; ++i) {
        __NOP();
    }
}

/** Один 16-битный обмен с опусканием/подъёмом CSn. */
static as5047p_status_t xfer(as5047p_t *dev, uint16_t tx, uint16_t *rx)
{
    uint16_t rx_buf = 0;
    HAL_StatusTypeDef hal;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    delay_cs();
    hal = HAL_SPI_TransmitReceive(dev->hspi, (uint8_t *)&tx, (uint8_t *)&rx_buf,
                                  1U, AS5047P_SPI_TIMEOUT);
    delay_cs();
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    delay_cs();

    if (rx != NULL) {
        *rx = rx_buf;
    }
    return (hal == HAL_OK) ? AS5047P_OK : AS5047P_ERR_SPI;
}

as5047p_status_t as5047p_read(as5047p_t *dev, uint16_t reg, uint16_t *value)
{
    as5047p_status_t st;
    uint16_t rx = 0;

    /* 1-й кадр: команда чтения. Ответ на неё относится к ПРЕДЫДУЩЕЙ команде. */
    st = xfer(dev, with_parity((uint16_t)((reg & AS5047P_DATA_Msk) | AS5047P_CMD_READ)), NULL);
    if (st != AS5047P_OK) {
        return st;
    }

    /* 2-й кадр: NOP — вытаскиваем данные запрошенного регистра. */
    st = xfer(dev, with_parity(AS5047P_REG_NOP | AS5047P_CMD_READ), &rx);
    if (st != AS5047P_OK) {
        return st;
    }

    /* Чётность всего ответного слова должна быть чётной. */
    if (parity(rx) != 0U) {
        return AS5047P_ERR_PARITY;
    }
    if ((rx & AS5047P_FRAME_EF) != 0U) {
        return AS5047P_ERR_FLAG;
    }

    if (value != NULL) {
        *value = (uint16_t)(rx & AS5047P_DATA_Msk);
    }
    return AS5047P_OK;
}

as5047p_status_t as5047p_write(as5047p_t *dev, uint16_t reg, uint16_t value)
{
    as5047p_status_t st;

    /* 1-й кадр: адрес, бит 14 = 0 (запись). */
    st = xfer(dev, with_parity((uint16_t)(reg & AS5047P_DATA_Msk)), NULL);
    if (st != AS5047P_OK) {
        return st;
    }

    /* 2-й кадр: данные. В ответ придёт новое содержимое регистра. */
    return xfer(dev, with_parity((uint16_t)(value & AS5047P_DATA_Msk)), NULL);
}

as5047p_status_t as5047p_read_angle(as5047p_t *dev, uint16_t *angle)
{
    return as5047p_read(dev, AS5047P_REG_ANGLECOM, angle);
}

uint32_t as5047p_raw_to_centideg(uint16_t raw)
{
    /* 16383 * 36000 = 589 788 000 — помещается в uint32_t без переполнения. */
    return ((uint32_t)(raw & AS5047P_DATA_Msk) * 36000UL) / AS5047P_RESOLUTION;
}

const char *as5047p_strerror(as5047p_status_t st)
{
    switch (st) {
    case AS5047P_OK:          return "OK";
    case AS5047P_ERR_SPI:     return "SPI transfer failed";
    case AS5047P_ERR_PARITY:  return "response parity error";
    case AS5047P_ERR_FLAG:    return "sensor error flag (EF)";
    default:                  return "unknown";
    }
}
