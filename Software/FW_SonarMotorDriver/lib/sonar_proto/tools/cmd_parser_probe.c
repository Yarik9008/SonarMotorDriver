/**
 * @file  cmd_parser_probe.c
 * @brief Хостовый зонд НАСТОЯЩЕГО парсера команд: строка на входе — разбор на выходе.
 *
 * Собирается на ПК обычным gcc вместе с НАСТОЯЩИМ src/cmd_parser.c (тот не
 * зависит ни от HAL, ни от периферии — только от string.h/stdlib.h/math.h),
 * поэтому проверяется именно тот код, который уходит в прошивку, а не его
 * пересказ регулярными выражениями или моделью на Python.
 *
 * Сделан по образцу FW_AS5047P_STM32F103C8/tools/filter_model.c, где так же
 * собирается настоящий src/angle_filter.c.
 *
 * Запуск — одной командой: tools/run_cmd_parser_probe.py (сборка + прогон).
 *
 * Протокол обмена (нарочно построчный и однозначный, чтобы его читал и
 * человек, и тест):
 *
 *   вход  — одна проверяемая строка на строку stdin; обратные слэши
 *           раскрываются (\\ \" \n \r \t \xNN), поэтому в проверку можно
 *           отдать и пробелы по краям, и табуляцию, и кавычку;
 *   выход — одна строка на каждый вход:
 *
 *       in="<вход как получен>" rc=0
 *       in="<вход как получен>" rc=1 type=CMD_SET_TARGET target=90
 *
 *           rc     — то, что вернул Cmd_Parse (0 = прошивка промолчит);
 *           type   — Cmd_Type при rc=1 (при rc=0 поля out прошивка не
 *                    смотрит вовсе, поэтому и здесь они не печатаются);
 *           далее  — значимые для этого типа поля Cmd_Result.
 *
 * Числа печатаются как %.9g от float: девяти значащих цифр хватает, чтобы
 * значение float читалось обратно без потерь, и не появляется хвост от
 * расширения до double.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmd_parser.h"

/* Приёмная строка зонда. Кольцо платы меньше (board.h UART_RX_RING_SIZE = 128),
 * но зонд проверяет Cmd_Parse, а не сборщик строки, и обязан уметь принять
 * заведомо длинный вход — иначе «очень длинная строка» проверялась бы обрезком. */
#define PROBE_LINE_MAX 4096

static const char *type_name(Cmd_Type t)
{
    switch (t) {
    case CMD_NONE:              return "CMD_NONE";
    case CMD_ENABLE:            return "CMD_ENABLE";
    case CMD_DISABLE:           return "CMD_DISABLE";
    case CMD_SET_TARGET:        return "CMD_SET_TARGET";
    case CMD_SET_KP:            return "CMD_SET_KP";
    case CMD_SET_KI:            return "CMD_SET_KI";
    case CMD_SET_KD:            return "CMD_SET_KD";
    case CMD_SET_VMAX:          return "CMD_SET_VMAX";
    case CMD_SET_ACCEL:         return "CMD_SET_ACCEL";
    case CMD_SET_OUTPUT_PERIOD: return "CMD_SET_OUTPUT_PERIOD";
    case CMD_SET_OUTPUT_MODE:   return "CMD_SET_OUTPUT_MODE";
    case CMD_SET_DEBUG:         return "CMD_SET_DEBUG";
    case CMD_SCAN:              return "CMD_SCAN";
    case CMD_STOP:              return "CMD_STOP";
    case CMD_CONTINUOUS:        return "CMD_CONTINUOUS";
    case CMD_SET_IRUN:          return "CMD_SET_IRUN";
    case CMD_SET_IHOLD:         return "CMD_SET_IHOLD";
    case CMD_SET_ICUR:          return "CMD_SET_ICUR";
    case CMD_SET_MSTEP:         return "CMD_SET_MSTEP";
    case CMD_GET_MCFG:          return "CMD_GET_MCFG";
    case CMD_DIAG:              return "CMD_DIAG";
    case CMD_SET_SYNC:          return "CMD_SET_SYNC";
    case CMD_GET_SYNC:          return "CMD_GET_SYNC";
    case CMD_SET_HOLD:          return "CMD_SET_HOLD";
    case CMD_GET_HOLD:          return "CMD_GET_HOLD";
    case CMD_GET_OUTPUT_MODE:   return "CMD_GET_OUTPUT_MODE";
    case CMD_UNKNOWN:           return "CMD_UNKNOWN";
    }
    /* Новое значение Cmd_Type, о котором зонд не знает: молчать нельзя —
     * иначе тест сверял бы несуществующее имя. */
    return "CMD_UNLISTED";
}

/** Значимые поля Cmd_Result для этого типа команды. */
static void print_fields(const Cmd_Result *r)
{
    switch (r->type) {
    case CMD_SET_TARGET:
        printf(" target=%.9g", (double)r->target);
        break;
    case CMD_CONTINUOUS:
        printf(" dir=%d", (int)r->continuous_dir);
        break;
    case CMD_SET_KP:
        printf(" kp=%.9g", (double)r->kp);
        break;
    case CMD_SET_KI:
        printf(" ki=%.9g", (double)r->ki);
        break;
    case CMD_SET_KD:
        printf(" kd=%.9g", (double)r->kd);
        break;
    case CMD_SET_VMAX:
        printf(" vmax=%.9g", (double)r->vmax);
        break;
    case CMD_SET_ACCEL:
        printf(" accel=%.9g", (double)r->accel);
        break;
    case CMD_SET_OUTPUT_PERIOD:
        printf(" op=%u", (unsigned)r->output_period_ms);
        break;
    case CMD_SET_OUTPUT_MODE:
        printf(" om=%u", (unsigned)r->output_mode);
        break;
    case CMD_SET_DEBUG:
        printf(" debug=%u", (unsigned)r->debug);
        break;
    case CMD_SCAN:
        printf(" start=%.9g end=%.9g step=%.9g delay=%u inf=%d",
               (double)r->scan_start, (double)r->scan_end, (double)r->scan_step,
               (unsigned)r->scan_delay_ms, (int)r->scan_infinite_dir);
        break;
    case CMD_SET_SYNC:
        printf(" sync=%u", (unsigned)r->sync_mode);
        break;
    case CMD_SET_HOLD:
        printf(" hold=%u", (unsigned)r->hold);
        break;
    case CMD_SET_IRUN:
        printf(" irun=%u", (unsigned)r->irun_ma);
        break;
    case CMD_SET_IHOLD:
        printf(" ihold=%u", (unsigned)r->ihold_ma);
        break;
    case CMD_SET_ICUR:
        printf(" irun=%u ihold=%u", (unsigned)r->irun_ma, (unsigned)r->ihold_ma);
        break;
    case CMD_SET_MSTEP:
        printf(" mstep=%u", (unsigned)r->microsteps);
        break;
    default:
        break;      /* команды без аргумента: en/dis/stop/mcfg/diag/запросы */
    }
}

/** Одна цифра шестнадцатеричного числа или -1. */
static int hex_digit(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/**
 * @brief Раскрывает обратные слэши на месте. Возвращает 0 при успехе, -1 на
 *        неизвестной последовательности (её молча съесть нельзя: проверка
 *        пойдёт не по той строке, которую задумал тест).
 *
 * Байт \x00 не поддерживается намеренно: Cmd_Parse принимает строку с нулевым
 * терминатором, и вставленный ноль просто обрезал бы её.
 */
static int unescape(char *s)
{
    char *dst = s;
    const char *src = s;

    while (*src) {
        if (*src != '\\') {
            *dst++ = *src++;
            continue;
        }
        src++;
        switch (*src) {
        case '\\': *dst++ = '\\'; src++; break;
        case '"':  *dst++ = '"';  src++; break;
        case 'n':  *dst++ = '\n'; src++; break;
        case 'r':  *dst++ = '\r'; src++; break;
        case 't':  *dst++ = '\t'; src++; break;
        case 'x': {
            int hi = hex_digit((unsigned char)src[1]);
            int lo = (hi < 0) ? -1 : hex_digit((unsigned char)src[2]);
            if (lo < 0 || (hi == 0 && lo == 0))
                return -1;
            *dst++ = (char)(hi * 16 + lo);
            src += 3;
            break;
        }
        default:
            return -1;
        }
    }
    *dst = '\0';
    return 0;
}

/** Обратная операция: печать строки в кавычках, пригодная для чтения назад. */
static void print_escaped(const char *s)
{
    for (; *s; s++) {
        unsigned char c = (unsigned char)*s;
        switch (c) {
        case '\\': fputs("\\\\", stdout); break;
        case '"':  fputs("\\\"", stdout); break;
        case '\n': fputs("\\n", stdout); break;
        case '\r': fputs("\\r", stdout); break;
        case '\t': fputs("\\t", stdout); break;
        default:
            if (c < 0x20 || c == 0x7f)
                printf("\\x%02x", c);
            else
                putchar((int)c);
            break;
        }
    }
}

int main(void)
{
    static char buf[PROBE_LINE_MAX];
    unsigned long no = 0;

    while (fgets(buf, (int)sizeof buf, stdin) != NULL) {
        size_t len = strlen(buf);
        int complete = (len > 0 && buf[len - 1] == '\n');
        Cmd_Result res;
        uint8_t rc;

        no++;
        if (!complete && !feof(stdin)) {
            fprintf(stderr, "cmd_parser_probe: строка %lu длиннее %d байт\n",
                    no, PROBE_LINE_MAX - 1);
            return 2;
        }
        while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
            buf[--len] = '\0';

        if (unescape(buf) != 0) {
            fprintf(stderr, "cmd_parser_probe: строка %lu: неизвестная "
                            "escape-последовательность\n", no);
            return 2;
        }

        memset(&res, 0, sizeof res);
        rc = Cmd_Parse(buf, &res);

        fputs("in=\"", stdout);
        print_escaped(buf);
        printf("\" rc=%u", (unsigned)rc);
        if (rc) {
            printf(" type=%s", type_name(res.type));
            print_fields(&res);
        }
        putchar('\n');
    }

    if (ferror(stdin)) {
        fprintf(stderr, "cmd_parser_probe: ошибка чтения stdin\n");
        return 2;
    }
    fflush(stdout);
    return 0;
}
