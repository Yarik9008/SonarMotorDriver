/*
 * cmd_parser.h — Парсер коротких USB-команд.
 *
 * Команды:
 *   en           — включить драйвер + PID (удержание текущей позиции)
 *   dis          — выключить
 *   t=X          — целевая позиция (градусы)
 *   t=+          — бесконечное вращение в положительном направлении
 *   t=-          — бесконечное вращение в отрицательном направлении
 *   kp=X, ki=X, kd=X — коэффициенты PID
 *   op=N         — период телеметрии (мс), 0 = выкл
 *   debug=0|1    — режим телеметрии: 0 = cp,ec; 1 = полная
 *   scan=s,e,st,d — сканирование сектора (start,end,step,delay_ms), zigzag до stop
 *   scan=s,+,st,d — бесконечное сканирование вперёд (start,step,delay_ms)
 *   scan=s,-,st,d — бесконечное сканирование назад  (start,step,delay_ms)
 *   stop         — остановить любое движение мотора
 *   DFU          — перезагрузка (обрабатывается в usb_cdc.c)
 */

#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <stdint.h>

/* Типы распознанных команд */
typedef enum {
    CMD_NONE,           /* не распознано / пусто */
    CMD_ENABLE,         /* en */
    CMD_DISABLE,        /* dis */
    CMD_SET_TARGET,     /* t=X (град) */
    CMD_SET_KP,         /* kp=X */
    CMD_SET_KI,         /* ki=X */
    CMD_SET_KD,         /* kd=X */
    CMD_SET_OUTPUT_PERIOD, /* op=N (мс) */
    CMD_SET_DEBUG,      /* debug=0|1 */
    CMD_SCAN,           /* scan=s,e,st,d или scan=s,+/-,st,d */
    CMD_STOP,           /* stop */
    CMD_CONTINUOUS,     /* t=+ или t=- (бесконечное вращение) */
    CMD_SET_IRUN,       /* irun <mA> */
    CMD_SET_IHOLD,      /* ihold <mA> */
    CMD_SET_ICUR,       /* icur <run_mA> <hold_mA> */
    CMD_SET_MSTEP,      /* mstep <value> */
    CMD_GET_MCFG,       /* mcfg */
    CMD_UNKNOWN         /* неизвестная команда */
} Cmd_Type;

/* Результат разбора команды */
typedef struct {
    Cmd_Type type;
    float kp, ki, kd;
    float target;           /* целевая позиция (град) */
    uint16_t output_period_ms;  /* период телеметрии, мс */
    uint8_t debug;          /* режим отладки: 0=краткий, 1=полный */
    float scan_start;       /* начало сканирования */
    float scan_end;         /* конец сканирования (zigzag) */
    float scan_step;        /* шаг сканирования */
    uint16_t scan_delay_ms; /* задержка между шагами */
    int8_t scan_infinite_dir;  /* 0=zigzag, +1=вперёд, -1=назад */
    int8_t continuous_dir;    /* +1=t=+, -1=t=- */
    uint16_t irun_ma;       /* ток движения, мА (irun/icur) */
    uint16_t ihold_ma;      /* ток удержания, мА (ihold/icur) */
    uint16_t microsteps;    /* микрошаг (mstep) */
} Cmd_Result;

/* Парсит строку line, заполняет out. Возвращает 1 при успехе, 0 при ошибке. */
uint8_t Cmd_Parse(const char *line, Cmd_Result *out);

#endif /* CMD_PARSER_H */
