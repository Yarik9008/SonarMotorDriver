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

typedef enum {
    CMD_NONE,
    CMD_ENABLE,
    CMD_DISABLE,
    CMD_SET_TARGET,
    CMD_SET_KP,
    CMD_SET_KI,
    CMD_SET_KD,
    CMD_SET_OUTPUT_PERIOD,
    CMD_SET_DEBUG,
    CMD_SCAN,
    CMD_STOP,
    CMD_CONTINUOUS,
    CMD_UNKNOWN
} Cmd_Type;

typedef struct {
    Cmd_Type type;
    float kp, ki, kd;
    float target;
    uint16_t output_period_ms;
    uint8_t debug;
    float scan_start;
    float scan_end;
    float scan_step;
    uint16_t scan_delay_ms;
    int8_t scan_infinite_dir; /* 0 = zigzag, +1 = scan,+, -1 = scan,- */
    int8_t continuous_dir;   /* +1 = t=+, -1 = t=- */
} Cmd_Result;

uint8_t Cmd_Parse(const char *line, Cmd_Result *out);

#endif /* CMD_PARSER_H */
