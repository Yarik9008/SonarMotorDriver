/**
 * @file cmd_parser.h
 * @brief Парсер коротких USB-команд.
 *
 * Команды:
 *   en           — включить драйвер + PID (удержание текущей позиции)
 *   dis          — выключить
 *   t=X          — целевая позиция (градусы)
 *   kp=X, ki=X, kd=X — коэффициенты PID
 *   op=N         — период телеметрии (мс), 0 = выкл
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
    CMD_UNKNOWN
} Cmd_Type;

typedef struct {
    Cmd_Type type;
    float kp, ki, kd;
    float target;
    uint16_t output_period_ms;
} Cmd_Result;

uint8_t Cmd_Parse(const char *line, Cmd_Result *out);

#endif /* CMD_PARSER_H */
