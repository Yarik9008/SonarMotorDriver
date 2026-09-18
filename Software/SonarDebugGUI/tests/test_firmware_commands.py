"""Страховка от расхождения GUI и прошивки: набор команд сверяется с исходником.

Протокол живёт в двух местах: парсер прошивки (cmd_parser.h / cmd_parser.c) и
слой sonar_gui/protocol.py, которым пользуются и GUI, и модель. Стоит добавить
в прошивку команду и забыть про GUI — расхождение всплывёт уже на стенде.
Поэтому тест читает исходник прошивки, вынимает из него список команд и
требует, чтобы каждая была представлена в таблице ниже: построителем из
protocol.py, клиентской валидацией и ответом модели.

Разбор нарочно устойчив к косметике: комментарии вырезаются, пробелы и
переносы строк не важны, значения вида `CMD_X = 5` допускаются, порядок
элементов не проверяется. Реагирует тест ровно на одно — появление или
исчезновение команды.
"""
from __future__ import annotations

import os
import re
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import _support     # noqa: F401,E402 - ставит sonar_gui на путь импорта
from _support import P, REPO_ROOT, Sim                 # noqa: E402

# Где искать парсер прошивки. Первый путь — исторический (каталог прошивки),
# второй — общая библиотека, куда модуль переехал. Если не нашли ни там, ни
# поиском по дереву, тест падает: значит, сверять больше не с чем.
HEADER_CANDIDATES = (
    os.path.join("Software", "FW_SonarMotorDriver", "include", "cmd_parser.h"),
    os.path.join("Software", "lib", "sonar_proto", "include", "cmd_parser.h"),
)
SOURCE_CANDIDATES = (
    os.path.join("Software", "FW_SonarMotorDriver", "src", "cmd_parser.c"),
    os.path.join("Software", "lib", "sonar_proto", "src", "cmd_parser.c"),
)
# Каталоги сборки и индексов clangd: там лежат копии и артефакты, не исходник.
SKIP_DIRS = (".pio", ".cache", ".git", "__pycache__")

# Значения Cmd_Type, которые командами не являются: пустая строка и «не
# распознано». Их в таблице покрытия быть не должно.
NON_COMMANDS = ("CMD_NONE", "CMD_UNKNOWN")


class Coverage:
    """Одна команда прошивки и то, чем она представлена в protocol.py."""

    def __init__(self, cmd_type, literals, builder, args, reply):
        self.cmd_type = cmd_type
        self.literals = literals    # строки, по которым её узнаёт Cmd_Parse()
        self.builder = builder      # имя построителя в protocol.py
        self.args = args
        self.reply = reply          # начало ответа модели прошивки

    def build(self) -> str:
        return getattr(P, self.builder)(*self.args)


COVERAGE = (
    Coverage("CMD_ENABLE",            ("en",),       "cmd_enable",        (), "ok:en"),
    Coverage("CMD_DISABLE",           ("dis",),      "cmd_disable",       (), "ok:dis"),
    Coverage("CMD_STOP",              ("stop",),     "cmd_stop",          (), "ok:stop"),
    Coverage("CMD_SET_TARGET",        ("t=",),       "cmd_target",     (90,), "ok:t=90.00"),
    Coverage("CMD_CONTINUOUS",  ("t=+", "t=-"),      "cmd_jog",       ("+",), "ok:t=+"),
    Coverage("CMD_SET_KP",            ("kp=",),      "cmd_kp",      (0.025,), "ok:kp=0.0250"),
    Coverage("CMD_SET_KI",            ("ki=",),      "cmd_ki",        (0.5,), "ok:ki=0.5000"),
    Coverage("CMD_SET_KD",            ("kd=",),      "cmd_kd",        (0.1,), "ok:kd=0.1000"),
    Coverage("CMD_SET_VMAX",          ("v=",),       "cmd_speed",     (600,), "ok:v=600.0"),
    Coverage("CMD_SET_ACCEL",         ("a=",),       "cmd_accel",    (2000,), "ok:a=2000.0"),
    Coverage("CMD_SET_OUTPUT_PERIOD", ("op=",),      "cmd_op",         (20,), "ok:op=20"),
    Coverage("CMD_SET_OUTPUT_MODE",   ("om=",),      "cmd_output_mode", (1,), "ok:om=1"),
    Coverage("CMD_SET_DEBUG",         ("debug=",),   "cmd_debug",    (True,), "ok:debug=1"),
    Coverage("CMD_SCAN",              ("scan=",),    "cmd_scan_sector",
             (0, 90, 5, 100), "ok:scan=0.00,90.00,5.00,100"),
    Coverage("CMD_SET_IRUN",          ("irun ",),    "cmd_irun",      (800,), "ok:irun=800"),
    Coverage("CMD_SET_IHOLD",         ("ihold ",),   "cmd_ihold",     (300,), "ok:ihold=300"),
    Coverage("CMD_SET_ICUR",          ("icur ",),    "cmd_icur", (700, 200), "ok:icur=700,200"),
    Coverage("CMD_SET_MSTEP",         ("mstep ",),   "cmd_mstep",      (16,), "ok:mstep=16"),
    Coverage("CMD_GET_MCFG",          ("mcfg",),     "cmd_mcfg",          (), "mode="),
    Coverage("CMD_DIAG",              ("diag",),     "cmd_diag",          (), "ok:diag"),
    Coverage("CMD_SET_SYNC",          ("sync=",),    "cmd_sync_mode",   (2,), "ok:sync=2"),
    Coverage("CMD_GET_SYNC",          ("sync",),     "cmd_sync_query",    (), "sync="),
    Coverage("CMD_SET_HOLD",          ("hold=",),    "cmd_hold",    (False,), "ok:hold=0"),
    # Запросы состояния без аргумента: ответ — строка данных без ok: (как sync)
    Coverage("CMD_GET_HOLD",          ("hold",),     "cmd_hold_query",    (), "hold="),
    Coverage("CMD_GET_OUTPUT_MODE",   ("om",),       "cmd_output_mode_query",
             (), "om="),
)


def _find(candidates, filename):
    """Путь к файлу прошивки: сначала известные места, потом поиск по дереву."""
    for rel in candidates:
        full = os.path.join(REPO_ROOT, rel)
        if os.path.isfile(full):
            return full
    software = os.path.join(REPO_ROOT, "Software")
    for root, dirs, files in os.walk(software):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        if filename in files:
            return os.path.join(root, filename)
    return None


def _read(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def _strip_comments(text: str) -> str:
    """Убирает /* ... */ и // ... — иначе в разбор попадут doxygen-пояснения."""
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def parse_cmd_types(header: str) -> list[str]:
    """Значения enum Cmd_Type в порядке объявления."""
    body = re.search(r"typedef\s+enum\s*\{(.*?)\}\s*Cmd_Type\s*;",
                     _strip_comments(header), re.S)
    if body is None:
        return []
    names = []
    for item in body.group(1).split(","):
        found = re.search(r"\b(CMD_[A-Za-z0-9_]+)", item)
        if found:
            names.append(found.group(1))
    return names


_ANCHOR = r'strn?cmp\s*\(\s*line\s*,\s*"((?:[^"\\]|\\.)*)"'


def parse_command_literals(source: str) -> list[str]:
    """Строки, по которым Cmd_Parse() узнаёт команду (strcmp/strncmp с line)."""
    return re.findall(_ANCHOR, _strip_comments(source))


def parse_arg_limits(source: str) -> dict:
    """Числовые границы аргумента внутри блока каждой команды Cmd_Parse().

    Блок команды — от её strcmp/strncmp до следующего: проверка вида
    `v < 0 || v > 3000` попадает именно в него, поэтому предел не путается с
    пределом соседней команды.
    """
    text = _strip_comments(source)
    anchors = [(m.start(), m.group(1)) for m in re.finditer(_ANCHOR, text)]
    limits = {}
    for i, (pos, literal) in enumerate(anchors):
        end = anchors[i + 1][0] if i + 1 < len(anchors) else len(text)
        limits[literal] = [int(v) for v
                           in re.findall(r"[<>]\s*(\d+)\b", text[pos:end])]
    return limits


class FirmwareCommandCoverageTests(unittest.TestCase):
    """Каждая команда прошивки должна быть представлена в protocol.py."""

    @classmethod
    def setUpClass(cls):
        cls.header_path = _find(HEADER_CANDIDATES, "cmd_parser.h")
        cls.source_path = _find(SOURCE_CANDIDATES, "cmd_parser.c")

    def test_header_is_found(self):
        """Без исходника прошивки сверять нечего — это и есть провал теста."""
        self.assertIsNotNone(
            self.header_path,
            "cmd_parser.h не найден: искали в " + ", ".join(HEADER_CANDIDATES) +
            " и поиском по Software/. Если файл переехал, поправьте "
            "HEADER_CANDIDATES в этом тесте.")

    def test_every_cmd_type_is_covered(self):
        """Новая команда в enum Cmd_Type обязана появиться и в таблице выше."""
        self.assertIsNotNone(self.header_path)
        names = parse_cmd_types(_read(self.header_path))
        self.assertGreater(len(names), 5, "enum Cmd_Type разобрать не удалось")
        for marker in NON_COMMANDS:
            self.assertIn(marker, names, "разбор enum сбился: нет " + marker)
        firmware = set(names) - set(NON_COMMANDS)
        covered = {c.cmd_type for c in COVERAGE}
        missing = sorted(firmware - covered)
        extra = sorted(covered - firmware)
        self.assertEqual(
            missing, [],
            "в прошивке есть команды, не покрытые protocol.py: " + ", ".join(missing))
        self.assertEqual(
            extra, [],
            "таблица покрытия ссылается на исчезнувшие команды: " + ", ".join(extra))

    def test_every_parser_literal_is_covered(self):
        """Команда могла появиться и без нового значения enum — сверяем строки."""
        if self.source_path is None:
            self.skipTest("cmd_parser.c не найден — сверять строки не с чем")
        literals = parse_command_literals(_read(self.source_path))
        self.assertGreater(len(literals), 5, "строки команд разобрать не удалось")
        firmware = set(literals)
        covered = {lit for c in COVERAGE for lit in c.literals}
        self.assertEqual(
            sorted(firmware - covered), [],
            "парсер прошивки узнаёт строки, которых нет в таблице покрытия")
        self.assertEqual(
            sorted(covered - firmware), [],
            "таблица покрытия ссылается на строки, которых в парсере уже нет")

    def test_builders_exist_and_match_literals(self):
        """Построитель protocol.py собирает строку, которую узнаёт парсер."""
        for cov in COVERAGE:
            with self.subTest(cmd=cov.cmd_type):
                self.assertTrue(hasattr(P, cov.builder),
                                f"в protocol.py нет построителя {cov.builder}()")
                cmd = cov.build()
                self.assertTrue(
                    any(cmd.startswith(lit) for lit in cov.literals),
                    f"{cov.builder}() дал '{cmd}', парсер ждёт {cov.literals}")

    def test_builders_pass_client_validation(self):
        """Собственный валидатор не должен блокировать то, что собрал GUI."""
        for cov in COVERAGE:
            with self.subTest(cmd=cov.cmd_type):
                ok, why = P.validate(cov.build())
                self.assertTrue(ok, f"{cov.cmd_type}: валидатор отверг команду ({why})")

    def test_model_answers_every_command(self):
        """Модель прошивки отвечает на каждую команду ожидаемой строкой."""
        for cov in COVERAGE:
            with self.subTest(cmd=cov.cmd_type):
                rep = Sim().send(cov.build())
                self.assertTrue(rep, f"{cov.cmd_type}: модель промолчала")
                self.assertTrue(
                    rep[0].startswith(cov.reply),
                    f"{cov.cmd_type}: ответ '{rep[0]}', ожидалось начало '{cov.reply}'")

    def test_parser_ranges_match_protocol_constants(self):
        """Границы аргументов в прошивке и в protocol.py — одни и те же числа.

        Разойдутся — GUI начнёт либо блокировать рабочие значения, либо
        отправлять то, на что плата молчит. Берём границы, записанные в
        cmd_parser.c литералами: токи, период телеметрии, режимы om= и sync=,
        паузу скана и аргумент mstep (последние два — uint16).

        У mstep границ две, и сверяется здесь именно ПАРСЕРНАЯ (65535, до
        приведения к uint16): набор 1/2/4…256 — это уже свойство чипа,
        которое проверяет обработчик команды прошивки, а не Cmd_Parse.
        """
        if self.source_path is None:
            self.skipTest("cmd_parser.c не найден")
        limits = parse_arg_limits(_read(self.source_path))
        cases = (
            ("irun ", P.CURRENT_MAX, "CURRENT_MAX"),
            ("ihold ", P.CURRENT_MAX, "CURRENT_MAX"),
            ("icur ", P.CURRENT_MAX, "CURRENT_MAX"),
            ("op=", P.OP_MAX, "OP_MAX"),
            ("om=", max(P.OUTPUT_MODE_VALUES), "OUTPUT_MODE_VALUES"),
            ("sync=", max(P.SYNC_MODE_VALUES), "SYNC_MODE_VALUES"),
            ("scan=", P.SCAN_DELAY_MAX, "SCAN_DELAY_MAX"),
            ("mstep ", P.MSTEP_ARG_MAX, "MSTEP_ARG_MAX"),
        )
        for literal, expect, name in cases:
            with self.subTest(cmd=literal):
                found = limits.get(literal)
                self.assertTrue(found,
                                f"в блоке команды '{literal}' не нашлось границ")
                self.assertEqual(
                    max(found), expect,
                    f"прошивка ограничивает '{literal}' числом {max(found)}, "
                    f"а protocol.{name} задаёт {expect}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
