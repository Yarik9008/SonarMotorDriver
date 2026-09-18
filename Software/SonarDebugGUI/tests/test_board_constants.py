"""Сверка констант хоста с исходниками прошивки.

sonar_gui/protocol.py объявлен единым источником правды для GUI и модели, но
сами числа в нём — копия board.h прошивки. Копия живёт отдельно и молча
расходится: правка PID_KP_DEFAULT или OUTPUT_PERIOD_MS_DEBUG_MIN на плате не
роняет ни один тест хоста, хотя после неё модель перестаёт быть моделью.

Поэтому здесь board.h (и два места, где число живёт не в board.h, а в коде
разбора команд) читаются как текст и сверяются с тем, что объявлено на хосте.
Тест не проверяет поведение — он проверяет, что хост и прошивка говорят об
одних и тех же величинах.

Разбор намеренно простой: #define с числом, числом с суффиксом U/L/f или
выражением из других #define. Чего разбор не берёт — перечислено в
NOT_CHECKED ниже, с причиной для каждого имени.
"""
from __future__ import annotations

import os
import re
import sys
import unittest
from unittest import mock

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import _support     # noqa: F401,E402 - ставит sonar_gui на путь импорта
from _support import P, REPO_ROOT, S                   # noqa: E402

BOARD_H = os.path.join(REPO_ROOT, "Software", "FW_SonarMotorDriver",
                       "include", "board.h")
CMD_PARSER_C = os.path.join(REPO_ROOT, "Software", "lib", "sonar_proto",
                            "src", "cmd_parser.c")
TMC2209_C = os.path.join(REPO_ROOT, "Software", "lib", "tmc2209",
                         "src", "tmc2209.c")

# Что сознательно НЕ сверяется с board.h и почему:
#  - OP_MIN/OP_MAX (0..65535) — не #define, а разрядность поля uint16_t
#    Cmd_Result.output_period_ms; отдельного имени в board.h нет.
#  - SCAN_DELAY_MIN/MAX — то же самое для Cmd_Result.scan_delay_ms.
#  - MAX_STEPS_PER_POLL — целочисленное деление C с округлением вверх;
#    на хосте величины нет, воспроизводить арифметику ради сверки незачем.
#  - simulator.SNAP_MIN_DEG (0.02) — литерал в теле main.c (порог доводочного
#    шага), имени в board.h у него нет.
#  - simulator.HOLD_DEFAULT (1) — стартовое значение переменной g_hold в
#    main.c, тоже не #define.
#  - JOG_FAST_DEG_S / JOG_SLOW_DEG_S — пресеты кнопок GUI, в прошивке их нет.
#  - ENCODER_ACCURACY_DEG, STALL_*, ENCODER_OUTLIER_* — на хосте не
#    используются (модель их поведение не воспроизводит, см. шапку
#    simulator.py), сверять нечего.
NOT_CHECKED = ("OP_MIN", "OP_MAX", "SCAN_DELAY_MIN", "SCAN_DELAY_MAX",
               "MAX_STEPS_PER_POLL", "SNAP_MIN_DEG", "HOLD_DEFAULT",
               "JOG_FAST_DEG_S", "JOG_SLOW_DEG_S")

# #define ИМЯ значение (без параметров: скобка сразу за именем — это макрос
# с аргументами, такие не нужны и не разбираются)
_RE_DEFINE = re.compile(
    r"^[ \t]*#[ \t]*define[ \t]+([A-Za-z_]\w*)[ \t]+(\S[^\n]*?)[ \t]*$", re.M)
_RE_BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.S)
_RE_LINE_COMMENT = re.compile(r"//[^\n]*")
# Приведение типа в духе (float)MAX_SPEED_DEG_S — для значения ничего не значит
_RE_CAST = re.compile(
    r"\((?:float|double|unsigned|signed|u?int(?:8|16|32|64)_t|int|long)\)")
# Суффиксы целых (123U, 4UL) и вещественных (0.05f) литералов
_RE_SUFFIX = re.compile(r"(?<=[0-9.])(?:[uU][lL]{0,2}|[lL]{1,2}[uU]?|[fF])\b")
_RE_IDENT = re.compile(r"[A-Za-z_]\w*")


def _strip_comments(text: str) -> str:
    return _RE_LINE_COMMENT.sub("", _RE_BLOCK_COMMENT.sub(" ", text))


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


class Defines:
    """Таблица #define одного заголовка с ленивым вычислением значений."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.raw = dict(_RE_DEFINE.findall(_strip_comments(_read(path))))

    def value(self, name: str, _seen: frozenset = frozenset()):
        """Число, стоящее за #define ИМЯ. KeyError — имени в заголовке нет.

        Выражения из других #define раскрываются рекурсивно. Деление в C
        целочисленное, в Python — нет; все используемые здесь выражения
        делятся нацело (POLL_FREQ_HZ = 1000/1), поэтому разницы не возникает,
        но новое выражение с остатком разбирать этим кодом нельзя.
        """
        if name in _seen:
            raise ValueError(f"{name}: циклическая ссылка в {self.path}")
        expr = _RE_SUFFIX.sub("", _RE_CAST.sub("", self.raw[name]))
        seen = _seen | {name}

        def expand(m: re.Match) -> str:
            return f"({self.value(m.group(), seen)!r})"

        expr = _RE_IDENT.sub(expand, expr)
        try:
            return eval(expr, {"__builtins__": {}}, {})     # noqa: S307
        except Exception as exc:                            # noqa: BLE001
            raise ValueError(
                f"{name}: не разобрано выражение {self.raw[name]!r} ({exc})")


BOARD = Defines(BOARD_H)

# (имя в board.h, значение на хосте, где это значение живёт)
BOARD_VS_HOST = (
    # Профиль движения и его границы
    ("MAX_SPEED_DEG_S",           P.MAX_SPEED_DEG_S,          "protocol.MAX_SPEED_DEG_S"),
    ("SPEED_MIN_DEG_S",           P.SPEED_MIN_DEG_S,          "protocol.SPEED_MIN_DEG_S"),
    ("ACCEL_MAX_DEG_S2",          P.ACCEL_MAX_DEG_S2,         "protocol.ACCEL_MAX_DEG_S2"),
    ("SPEED_DEFAULT_DEG_S",       P.DEFAULTS.vmax,            "Defaults.vmax"),
    ("ACCEL_DEFAULT_DEG_S2",      P.DEFAULTS.accel,           "Defaults.accel"),
    # ПИД
    ("PID_KP_DEFAULT",            P.DEFAULTS.kp,              "Defaults.kp"),
    ("PID_KI_DEFAULT",            P.DEFAULTS.ki,              "Defaults.ki"),
    ("PID_KD_DEFAULT",            P.DEFAULTS.kd,              "Defaults.kd"),
    ("PID_DEADBAND_DEG",          S.DEADBAND_DEG,             "simulator.DEADBAND_DEG"),
    # Телеметрия
    ("OUTPUT_PERIOD_MS_DEFAULT",  P.DEFAULTS.op_ms,           "Defaults.op_ms"),
    ("OUTPUT_PERIOD_MS_DEBUG_MIN", S.OUTPUT_PERIOD_MS_DEBUG_MIN,
     "simulator.OUTPUT_PERIOD_MS_DEBUG_MIN"),
    ("TELEMETRY_DEBUG_DEFAULT",   P.DEFAULTS.debug,           "Defaults.debug"),
    ("TELEMETRY_MODE_DEFAULT",    S.TELEM_MODE_DEFAULT,       "simulator.TELEM_MODE_DEFAULT"),
    # Драйвер TMC2209
    ("TMC2209_IRUN_MA",           P.DEFAULTS.irun,            "Defaults.irun"),
    ("TMC2209_IHOLD_MA",          P.DEFAULTS.ihold,           "Defaults.ihold"),
    ("TMC2209_MICROSTEPS",        P.DEFAULTS.microsteps,      "Defaults.microsteps"),
    ("TMC2209_MICROSTEPS",        P.TMC2209_MICROSTEPS_DEF,   "protocol.TMC2209_MICROSTEPS_DEF"),
    # Механика и энкодер
    ("MOTOR_FULL_STEPS_REV",      P.MOTOR_FULL_STEPS_REV,     "protocol.MOTOR_FULL_STEPS_REV"),
    ("MOTOR_STEPS_PER_REV",       P.MOTOR_STEPS_PER_REV,      "protocol.MOTOR_STEPS_PER_REV"),
    ("ENCODER_COUNTS_REV",        P.ENCODER_COUNTS_REV,       "protocol.ENCODER_COUNTS_REV"),
    # Стартовая диагностика энкодера (по ней конформанс-тест читает enc:ok)
    ("ENCODER_DIAG_SAMPLES",      P.ENCODER_DIAG_SAMPLES,     "protocol.ENCODER_DIAG_SAMPLES"),
    ("ENCODER_DIAG_MIN_OK",       P.ENCODER_DIAG_MIN_OK,      "protocol.ENCODER_DIAG_MIN_OK"),
    ("ENCODER_DIAG_MAX_SPREAD_DEG", P.ENCODER_DIAG_MAX_SPREAD_DEG,
     "protocol.ENCODER_DIAG_MAX_SPREAD_DEG"),
    # Связь с хостом
    ("UART_BAUDRATE",             P.BAUD,                     "protocol.BAUD"),
)


class BoardHeaderParserTests(unittest.TestCase):
    """Сначала убеждаемся, что разбор board.h вообще работает."""

    def test_header_is_found_and_parsed(self):
        self.assertTrue(os.path.isfile(BOARD_H), BOARD_H)
        self.assertGreater(len(BOARD.raw), 50,
                           "в board.h нашлось подозрительно мало #define")

    def test_comments_do_not_leak_into_values(self):
        """Значение берётся до комментария: «600U /* ток */» это 600."""
        self.assertEqual(BOARD.value("TMC2209_IRUN_MA"), 600)

    def test_expressions_are_expanded(self):
        """Выражение из других #define раскрывается, а не пропускается."""
        self.assertEqual(BOARD.value("MOTOR_STEPS_PER_REV"),
                         BOARD.value("MOTOR_FULL_STEPS_REV")
                         * BOARD.value("TMC2209_MICROSTEPS"))
        self.assertEqual(BOARD.value("SPEED_DEFAULT_DEG_S"),
                         BOARD.value("MAX_SPEED_DEG_S"))

    def test_unknown_name_is_an_error(self):
        """Переименовали константу в board.h — сверка обязана упасть, а не молчать."""
        with self.assertRaises(KeyError):
            BOARD.value("PID_KP_DEFAULT_RENAMED")


class BoardConstantsTests(unittest.TestCase):
    """Числа хоста = числа board.h."""

    def test_defines_match_host_values(self):
        for name, host_value, where in BOARD_VS_HOST:
            with self.subTest(define=name, host=where):
                fw_value = BOARD.value(name)
                self.assertAlmostEqual(
                    float(fw_value), float(host_value), places=6,
                    msg=f"board.h {name}={fw_value}, а {where}={host_value}")

    def test_control_period_matches_poll_rate(self):
        """Такт ПИД модели = период главного цикла прошивки (POLL_FREQ_HZ)."""
        self.assertAlmostEqual(S.CONTROL_DT_S, 1.0 / BOARD.value("POLL_FREQ_HZ"),
                               places=9)

    def test_pid_start_bounds_come_from_the_speed_ceiling(self):
        """Стартовые границы насыщения ПИД = MAX_DEG_PER_TICK прошивки.

        Прошивка задаёт их статическим инициализатором g_pid (main.c:416)
        через MAX_DEG_PER_TICK = MAX_SPEED_DEG_S / POLL_FREQ_HZ и пересчитывает
        только в обработчике v= (main.c:1519-1520). Взять их на старте от
        vmax — значит опереться на равенство SPEED_DEFAULT_DEG_S ==
        MAX_SPEED_DEG_S, которое в board.h ничем не закреплено: разведи эти
        константы — и модель молча разойдётся с платой.

        Тест стережёт три вещи:
        1. число у модели равно MAX_SPEED_DEG_S/POLL_FREQ_HZ из board.h —
           значит правка board.h без правки хоста заметна;
        2. стартовый vmax идёт от ДРУГОЙ константы, SPEED_DEFAULT_DEG_S;
        3. границы берутся именно от потолка скорости, а не от vmax, —
           и это проверяется на разведённых константах: пока обе равны 1200,
           первых двух проверок мало, модель с `pid_out_max = vmax·DT` прошла
           бы их насквозь. Поэтому потолок подменяется в самом модуле
           протокола ДО создания модели: подмена в конструкторе тогда даёт
           другое число. Через поле fw.vmax (как в поведенческой паре
           test_simulator.PidSaturationTests) путь инициализации не
           закрывается — оно двигается уже после конструктора.
        """
        fw = S.FirmwareSimulator()
        self.assertAlmostEqual(
            fw.pid_out_max,
            BOARD.value("MAX_SPEED_DEG_S") / BOARD.value("POLL_FREQ_HZ"),
            places=9,
            msg="границы ПИД у модели разошлись с MAX_DEG_PER_TICK прошивки")
        # А стартовый предел скорости — из другой константы board.h: две
        # величины модели читаются из двух разных #define, и разведи их
        # прошивка, обе останутся верными без правки модели.
        self.assertAlmostEqual(
            fw.vmax, float(BOARD.value("SPEED_DEFAULT_DEG_S")), places=6,
            msg="стартовый vmax модели должен идти от SPEED_DEFAULT_DEG_S")

        # Мир, в котором board.h развёл константы: потолок вдвое выше
        # умолчания скорости. Модель обязана взять потолок.
        ceiling = float(BOARD.value("MAX_SPEED_DEG_S")) * 2.0 + 1.0
        with mock.patch.object(P, "MAX_SPEED_DEG_S", ceiling):
            fw = S.FirmwareSimulator()
        self.assertAlmostEqual(
            fw.pid_out_max, ceiling * S.CONTROL_DT_S, places=9,
            msg="конструктор модели взял границы ПИД не от MAX_SPEED_DEG_S")
        self.assertAlmostEqual(
            fw.vmax, float(P.DEFAULTS.vmax), places=6,
            msg="стартовый vmax не должен зависеть от потолка скорости")

    def test_error_codes_cover_the_whole_enum(self):
        """Легенда ec перечисляет ровно те коды, что объявлены в ErrCode."""
        body = re.search(r"typedef enum\s*\{(.*?)\}\s*ErrCode\s*;",
                         _strip_comments(_read(BOARD_H)), re.S)
        self.assertIsNotNone(body, "в board.h не найден enum ErrCode")
        names = [m.group(1) for m in
                 re.finditer(r"([A-Za-z_]\w*)\s*(?:=\s*\d+\s*)?,", body.group(1))]
        self.assertIn("ERR_OK", names)
        codes = {i: n for i, n in enumerate(names) if n != "ERR_COUNT"}
        self.assertEqual(sorted(P.EC_LEGEND), sorted(codes),
                         f"коды ErrCode прошивки: {codes}")

    def test_defaults_are_not_silently_reordered(self):
        """Стартовая цель прошивки — 0° (board.h STARTUP_TARGET_OFFSET_DEG)."""
        self.assertAlmostEqual(P.DEFAULTS.target_deg,
                               BOARD.value("STARTUP_TARGET_OFFSET_DEG"), places=6)


class CommandParserConstantsTests(unittest.TestCase):
    """Числа, которые живут не в board.h, а прямо в разборе команд."""

    def test_current_limits_match_cmd_parser(self):
        """Границы irun/ihold/icur — литералы в cmd_parser.c, не #define.

        Берём их из тела разбора каждой из трёх команд (от strncmp до первого
        `return 1;`), иначе в выборку попали бы границы соседних команд.
        """
        src = _strip_comments(_read(CMD_PARSER_C))
        bounds: dict[str, set[int]] = {}
        for cmd in ("irun", "ihold", "icur"):
            block = re.search(
                r'strncmp\(line,\s*"' + cmd + r' ".*?return 1;', src, re.S)
            self.assertIsNotNone(block, f"в cmd_parser.c не найден разбор {cmd}")
            bounds[cmd] = {int(v) for v in re.findall(
                r"[a-z]\s*<\s*0\s*\|\|\s*[a-z]\s*>\s*(\d+)", block.group())}
            self.assertTrue(bounds[cmd], f"{cmd}: границы тока не найдены")
        found = set().union(*bounds.values())
        self.assertEqual(found, {P.CURRENT_MAX},
                         f"границы тока в cmd_parser.c: {bounds}")
        self.assertEqual(P.CURRENT_MIN, 0, "нижняя граница в cmd_parser.c — «< 0»")

    def test_microstep_set_matches_driver(self):
        """MSTEP_VALUES = ровно те значения, что принимает microsteps_to_mres()."""
        src = _strip_comments(_read(TMC2209_C))
        body = re.search(r"microsteps_to_mres\s*\([^)]*\)\s*\{(.*?)\n\}", src, re.S)
        self.assertIsNotNone(body, "в tmc2209.c не найдена microsteps_to_mres()")
        cases = {int(v) for v in re.findall(r"case\s+(\d+)\s*:", body.group(1))}
        self.assertEqual(cases, set(P.MSTEP_VALUES))


if __name__ == "__main__":
    unittest.main(verbosity=2)
