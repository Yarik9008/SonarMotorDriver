/**
 * @file  syscalls.c
 * @brief Заглушки newlib + перенаправление stdout (printf) в USB CDC
 *        (или в USART1, если собрано с -D LOG_USE_UART=1).
 */
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "stm32f1xx_hal.h"

#if LOG_USE_UART
extern UART_HandleTypeDef huart1;
#else
#include "usbd_cdc_if.h"
#endif

#undef errno
extern int errno;

/* --- Вывод: всё, что печатает printf/putchar, уходит в лог-канал --- */
int _write(int file, char *ptr, int len)
{
    (void)file;

    if (len <= 0) {
        return 0;
    }

#if LOG_USE_UART
    if (HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY) != HAL_OK) {
        errno = EIO;
        return -1;
    }
    return len;
#else
    if (usb_cdc_write((const uint8_t *)ptr, (uint16_t)len) < 0) {
        /* Порт на ПК не открыт: строку теряем, но не блокируем опрос датчика.
           Для printf это успех — иначе newlib начнёт помечать поток как сбойный. */
        return len;
    }
    return len;
#endif
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;   /* ввод не используется */
}

/* --- Куча для newlib (нужна printf'у) --- */
caddr_t _sbrk(int incr)
{
    extern char end asm("end");   /* символ из линкер-скрипта: конец .bss */
    static char *heap_end = NULL;
    char *prev_heap_end;

    if (heap_end == NULL) {
        heap_end = &end;
    }
    prev_heap_end = heap_end;

    /* граница кучи — текущая вершина стека */
    if (heap_end + incr > (char *)__get_MSP()) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }
    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

/* --- Минимальные заглушки остальных системных вызовов --- */
int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    (void)status;
    for (;;) {
    }
}
