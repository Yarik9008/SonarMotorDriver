/**
 * @file cmd_parser.h
 * @brief Парсер текстовых команд, поступающих по UART/USB.
 *
 * Модуль преобразует текстовые строки в структурированные команды управления
 * мотором, параметрами ПИД-регулятора, режимами сканирования и настройками драйвера.
 *
 * Список поддерживаемых команд:
 * - en / dis : включение/выключение силовой части и ПИД.
 * - t=X      : установка целевого угла в градусах.
 * - t=+ / t- : непрерывное вращение.
 * - kp/ki/kd=X : настройка коэффициентов регулятора.
 * - v=X      : предел скорости движения, град/с.
 * - a=X      : предел ускорения, град/с² (0 = без ограничения).
 * - op=N     : период выдачи телеметрии в мс (0 - выкл).
 * - scan=... : запуск автоматического сканирования (секторного или непрерывного).
 * - irun/ihold/mstep : прямая настройка параметров чипа TMC2209.
 * - diag     : диагностика энкодера (серия чтений, ответ enc:ok / err:enc).
 * - sync=N   : источник перехода к следующей точке скана (0=таймер delay,
 *              1=фронт SYNC_IN, 2=фронт SYNC_IN или delay как тайм-аут).
 * - sync     : запрос состояния синхронизации (режим, уровни пинов, счётчик фронтов).
 */

#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <stdint.h>

/**
 * @brief Типы распознаваемых команд.
 */
typedef enum {
    CMD_NONE,               ///< Пустая строка или ошибка
    CMD_ENABLE,             ///< Включить мотор (en)
    CMD_DISABLE,            ///< Выключить мотор (dis)
    CMD_SET_TARGET,         ///< Установка позиции (t=X)
    CMD_SET_KP,             ///< Настройка Kp (kp=X)
    CMD_SET_KI,             ///< Настройка Ki (ki=X)
    CMD_SET_KD,             ///< Настройка Kd (kd=X)
    CMD_SET_VMAX,           ///< Предел скорости (v=X, град/с)
    CMD_SET_ACCEL,          ///< Предел ускорения (a=X, град/с²; 0 = выкл)
    CMD_SET_OUTPUT_PERIOD,  ///< Период телеметрии (op=N)
    CMD_SET_DEBUG,          ///< Режим отладки (debug=0|1)
    CMD_SCAN,               ///< Режим сканирования (scan=...)
    CMD_STOP,               ///< Экстренная остановка (stop)
    CMD_CONTINUOUS,         ///< Непрерывное вращение (t=+/t=-)
    CMD_SET_IRUN,           ///< Ток движения (irun mA)
    CMD_SET_IHOLD,          ///< Ток удержания (ihold mA)
    CMD_SET_ICUR,           ///< Установка обоих токов (icur mA mA)
    CMD_SET_MSTEP,          ///< Микрошаг (mstep N)
    CMD_GET_MCFG,           ///< Запрос конфигурации TMC2209 (mcfg)
    CMD_DIAG,               ///< Диагностика энкодера (diag)
    CMD_SET_SYNC,           ///< Режим синхронизации скана (sync=N)
    CMD_GET_SYNC,           ///< Запрос состояния синхронизации (sync)
    CMD_UNKNOWN             ///< Команда не распознана
} Cmd_Type;

/* Результат разбора команды */
typedef struct {
    Cmd_Type type;
    float kp, ki, kd;
    float vmax;             /* предел скорости, град/с (v=) */
    float accel;            /* предел ускорения, град/с² (a=) */
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
    uint8_t sync_mode;      /* режим синхронизации скана (sync=): 0..2 */
} Cmd_Result;

/* Парсит строку line, заполняет out. Возвращает 1 при успехе, 0 при ошибке. */
uint8_t Cmd_Parse(const char *line, Cmd_Result *out);

#endif /* CMD_PARSER_H */
