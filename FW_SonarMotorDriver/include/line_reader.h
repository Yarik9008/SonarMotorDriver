/* line_reader.h — Общая логика разбора строк из кольцевых буферов. */

#ifndef LINE_READER_H
#define LINE_READER_H

#include <stdint.h>

/**
 * Извлекает строку из кольцевого буфера.
 * - Обрабатывает \r, \n, \r\n.
 * - Автоматически пропускает пустые строки.
 * - Предотвращает зависание буфера (сбрасывает при переполнении).
 *
 * @param ring       Кольцевой буфер
 * @param ring_size  Размер буфера
 * @param head       Текущий индекс записи (head)
 * @param tail       Указатель на текущий индекс чтения (tail)
 * @param buf        Буфер для извлечённой строки
 * @param buf_size   Размер выходного буфера
 * @return Длина извлечённой строки (>0). Если готовой строки нет, возвращает 0.
 */
uint16_t line_reader_extract(const uint8_t *ring, uint16_t ring_size, 
                             uint16_t head, uint16_t *tail, 
                             char *buf, uint16_t buf_size);

#endif /* LINE_READER_H */
