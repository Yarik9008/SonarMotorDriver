/**
 * @file cmd_parser.c
 * @brief Реализация парсера команд.
 */

#include "cmd_parser.h"
#include <string.h>
#include <stdlib.h>

static int parse_float(const char **p, float *out)
{
    char *end;
    float v = strtof(*p, &end);
    if (end == *p)
        return -1;
    *p = end;
    *out = v;
    return 0;
}

static int parse_int(const char **p, int32_t *out)
{
    char *end;
    long v = strtol(*p, &end, 10);
    if (end == *p)
        return -1;
    *p = end;
    *out = (int32_t)v;
    return 0;
}

uint8_t Cmd_Parse(const char *line, Cmd_Result *out)
{
    if (!line || !out)
        return 0;

    out->type = CMD_NONE;
    out->kp = out->ki = out->kd = 0.0f;
    out->target = 0.0f;

    if (strcmp(line, "en") == 0) {
        out->type = CMD_ENABLE;
        return 1;
    }
    if (strcmp(line, "dis") == 0) {
        out->type = CMD_DISABLE;
        return 1;
    }

    /* t=X — целевая позиция (градусы) */
    if (strncmp(line, "t=", 2) == 0) {
        const char *p = line + 2;
        float t;
        if (parse_float(&p, &t) != 0)
            return 0;
        out->type = CMD_SET_TARGET;
        out->target = t;
        return 1;
    }

    /* kp=X — пропорциональный коэффициент */
    if (strncmp(line, "kp=", 3) == 0) {
        const char *p = line + 3;
        if (parse_float(&p, &out->kp) != 0)
            return 0;
        out->type = CMD_SET_KP;
        return 1;
    }
    /* ki=X — интегральный */
    if (strncmp(line, "ki=", 3) == 0) {
        const char *p = line + 3;
        if (parse_float(&p, &out->ki) != 0)
            return 0;
        out->type = CMD_SET_KI;
        return 1;
    }
    /* kd=X — дифференциальный */
    if (strncmp(line, "kd=", 3) == 0) {
        const char *p = line + 3;
        if (parse_float(&p, &out->kd) != 0)
            return 0;
        out->type = CMD_SET_KD;
        return 1;
    }

    /* op=N — период телеметрии */
    if (strncmp(line, "op=", 3) == 0) {
        const char *p = line + 3;
        int32_t v;
        if (parse_int(&p, &v) != 0 || v < 0 || v > 65535)
            return 0;
        out->type = CMD_SET_OUTPUT_PERIOD;
        out->output_period_ms = (uint16_t)v;
        return 1;
    }

    out->type = CMD_UNKNOWN;
    return 0;
}
