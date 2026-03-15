/* line_reader.h — Общая логика разбора строк из кольцевых буферов. */

#ifndef LINE_READER_H
#define LINE_READER_H

#include <stdint.h>

/* Извлекает строку из кольцевого буфера.
 * Обрабатывает \r, \n, \r\n; пропускает пустые строки.
 * При переполнении (строка без EOL) постепенно вытесняет старые байты («slow-drain»), 
 * чтобы буфер не завис и новые данные могли быть приняты.
 *
 * ring     — кольцевой буфер данных
 * ring_size — размер буфера
 * head     — текущий индекс записи (volatile)
 * tail     — указатель на индекс чтения (обновляется при извлечении)
 * buf      — буфер для результата
 * buf_size — размер buf
 * Возврат: длина строки (>0) или 0, если нет готовой строки. */
uint16_t line_reader_extract(const uint8_t *ring, uint16_t ring_size, 
                             uint16_t head, uint16_t *tail, 
                             char *buf, uint16_t buf_size);

#endif /* LINE_READER_H */
