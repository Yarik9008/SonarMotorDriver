/**
 * @file line_reader.c
 * @brief Реализация логики извлечения строк.
 */

#include "line_reader.h"

uint16_t line_reader_extract(const uint8_t *ring, uint16_t ring_size, 
                             uint16_t head, uint16_t *tail, 
                             char *buf, uint16_t buf_size)
{
    if (buf_size == 0) return 0;

    while (1) {
        uint16_t t = *tail;
        uint16_t len = 0;
        uint8_t found_eol = 0;
        uint16_t eol_t = t;

        if (t == head) return 0;

        /* Ищем конец строки */
        while (t != head) {
            uint8_t c = ring[t];
            t = (t + 1) % ring_size;
            
            if (c == '\r' || c == '\n') {
                found_eol = 1;
                eol_t = t;
                /* Обработка \r\n */
                if (c == '\r' && t != head && ring[t] == '\n') {
                    eol_t = (t + 1) % ring_size;
                }
                break;
            }
            len++;
        }

        if (!found_eol) {
            /* Защита от переполнения: если в буфере больше нет места для поиска EOL */
            if (len >= ring_size - 1) {
                /* Сдвигаем tail на один байт, чтобы освободить место для новых данных */
                *tail = (*tail + 1) % ring_size;
            }
            return 0;
        }

        /* Извлекаем строку */
        t = *tail;
        uint16_t out_len = 0;
        
        while (t != eol_t) {
            uint8_t c = ring[t];
            t = (t + 1) % ring_size;
            if (c == '\r' || c == '\n') continue;
            
            if (out_len < buf_size - 1) {
                buf[out_len++] = (char)c;
            }
        }

        *tail = eol_t;
        buf[out_len] = '\0';
        
        /* Если получили непустую строку — возвращаем */
        if (out_len > 0) {
            return out_len;
        }
        /* Пустые строки пропускаем и ищем следующую */
    }
}
