/* cmd_parser.c — Парсер текстовых команд.
 *
 * Форматы: en, dis, stop, t=X, t=+/-, kp=/ki=/kd=X, op=N, debug=0|1,
 *          scan=start,end,step,delay или scan=start,+/-,step,delay,
 *          irun <mA>, ihold <mA>, icur <run> <hold>, mstep <value>, mcfg.
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

static const char *skip_spaces(const char *p)
{
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

uint8_t Cmd_Parse(const char *line, Cmd_Result *out)
{
    if (!line || !out || line[0] == '\0')
        return 0;

    out->type = CMD_NONE;
    out->kp = out->ki = out->kd = 0.0f;
    out->target = 0.0f;
    out->output_period_ms = 0;
    out->debug = 0;
    out->scan_start = out->scan_end = out->scan_step = 0.0f;
    out->scan_delay_ms = 0;
    out->scan_infinite_dir = 0;
    out->continuous_dir = 0;
    out->irun_ma = 0;
    out->ihold_ma = 0;
    out->microsteps = 0;

    if (strcmp(line, "en") == 0) {
        out->type = CMD_ENABLE;
        return 1;
    }
    if (strcmp(line, "dis") == 0) {
        out->type = CMD_DISABLE;
        return 1;
    }
    if (strcmp(line, "stop") == 0) {
        out->type = CMD_STOP;
        return 1;
    }

    /* t=+ / t=- — бесконечное вращение */
    if (strcmp(line, "t=+") == 0) {
        out->type = CMD_CONTINUOUS;
        out->continuous_dir = +1;
        return 1;
    }
    if (strcmp(line, "t=-") == 0) {
        out->type = CMD_CONTINUOUS;
        out->continuous_dir = -1;
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

    /* debug=0|1 — режим телеметрии */
    if (strncmp(line, "debug=", 6) == 0) {
        const char *p = line + 6;
        int32_t v;
        if (parse_int(&p, &v) != 0 || (v != 0 && v != 1))
            return 0;
        out->type = CMD_SET_DEBUG;
        out->debug = (uint8_t)v;
        return 1;
    }

    /* scan=start,end,step,delay — сканирование сектора */
    /* scan=start,+,step,delay  — бесконечное сканирование вперёд  */
    /* scan=start,-,step,delay  — бесконечное сканирование назад   */
    if (strncmp(line, "scan=", 5) == 0) {
        const char *p = line + 5;
        float s, st;
        int32_t d;
        if (parse_float(&p, &s) != 0 || *p++ != ',')
            return 0;

        int8_t inf_dir = 0;
        float e = 0.0f;
        if ((*p == '+' || *p == '-') && *(p + 1) == ',') {
            inf_dir = (*p == '+') ? +1 : -1;
            p += 2;
        } else {
            if (parse_float(&p, &e) != 0 || *p++ != ',')
                return 0;
        }

        if (parse_float(&p, &st) != 0 || *p++ != ',')
            return 0;
        if (parse_int(&p, &d) != 0 || d < 0 || d > 65535)
            return 0;
        out->type = CMD_SCAN;
        out->scan_start = s;
        out->scan_end = e;
        out->scan_step = st;
        out->scan_delay_ms = (uint16_t)d;
        out->scan_infinite_dir = inf_dir;
        return 1;
    }

    /* mcfg — вывод текущей конфигурации драйвера */
    if (strcmp(line, "mcfg") == 0) {
        out->type = CMD_GET_MCFG;
        return 1;
    }

    /* irun <mA> — установить ток движения */
    if (strncmp(line, "irun ", 5) == 0) {
        const char *p = skip_spaces(line + 5);
        int32_t v;
        if (parse_int(&p, &v) != 0 || v < 0 || v > 3000)
            return 0;
        out->type    = CMD_SET_IRUN;
        out->irun_ma = (uint16_t)v;
        return 1;
    }

    /* ihold <mA> — установить ток удержания */
    if (strncmp(line, "ihold ", 6) == 0) {
        const char *p = skip_spaces(line + 6);
        int32_t v;
        if (parse_int(&p, &v) != 0 || v < 0 || v > 3000)
            return 0;
        out->type     = CMD_SET_IHOLD;
        out->ihold_ma = (uint16_t)v;
        return 1;
    }

    /* icur <run_mA> <hold_mA> — установить оба тока */
    if (strncmp(line, "icur ", 5) == 0) {
        const char *p = skip_spaces(line + 5);
        int32_t r, h;
        if (parse_int(&p, &r) != 0 || r < 0 || r > 3000)
            return 0;
        p = skip_spaces(p);
        if (parse_int(&p, &h) != 0 || h < 0 || h > 3000)
            return 0;
        out->type     = CMD_SET_ICUR;
        out->irun_ma  = (uint16_t)r;
        out->ihold_ma = (uint16_t)h;
        return 1;
    }

    /* mstep <value> — установить микрошаг */
    if (strncmp(line, "mstep ", 6) == 0) {
        const char *p = skip_spaces(line + 6);
        int32_t v;
        if (parse_int(&p, &v) != 0 || v < 0)
            return 0;
        out->type       = CMD_SET_MSTEP;
        out->microsteps = (uint16_t)v;
        return 1;
    }

    out->type = CMD_UNKNOWN;
    return 1;
}
