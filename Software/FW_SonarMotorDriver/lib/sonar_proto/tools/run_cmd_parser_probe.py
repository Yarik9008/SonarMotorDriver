#!/usr/bin/env python3
"""Сборка и прогон хостового зонда парсера команд — одной командой:

    python tools/run_cmd_parser_probe.py            # таблица по корпусу CORPUS
    python tools/run_cmd_parser_probe.py --stdin    # строки берутся из stdin
    python tools/run_cmd_parser_probe.py --out DIR  # двоичный файл остаётся в DIR

Собирает НАСТОЯЩИЙ src/cmd_parser.c обычным хостовым gcc вместе с
tools/cmd_parser_probe.c и печатает, как парсер прошивки разобрал каждую
строку. Тот же путь, что у FW_AS5047P_STM32F103C8/tools/run_filter_model.sh,
где так же собирается настоящий src/angle_filter.c.

Почему Python, а не .sh, как у фильтра: этот раннер вызывает не только
человек, но и юнит-тест SonarDebugGUI (tests/test_cmd_parser_host.py), а он
идёт и на Windows, где /bin/sh может отсутствовать. Флаги сборки при этом
остаются в одном месте — если бы их продублировал ещё и shell-обёртка, они
разъехались бы первым же изменением.

Модуль рассчитан и на импорт: build() собирает зонд, run() прогоняет через
него список строк и возвращает разобранные результаты (Result).
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
LIB_DIR = os.path.dirname(_HERE)                       # .../lib/sonar_proto
INCLUDE_DIR = os.path.join(LIB_DIR, "include")
SRC_PARSER = os.path.join(LIB_DIR, "src", "cmd_parser.c")
SRC_PROBE = os.path.join(_HERE, "cmd_parser_probe.c")

# -Werror здесь, а не в library.json: библиотека собирается в прошивку чужой
# toolchain'ой (arm-none-eabi), и поднимать там предупреждения до ошибок —
# отдельное решение. На хосте своя сборка и свой компилятор, поэтому любое
# предупреждение в проверяемом коде обязано валить прогон.
CFLAGS = ("-std=c11", "-O1", "-Wall", "-Wextra", "-Werror")

# Кандидаты на роль компилятора, если CC не задан переменной окружения.
CC_CANDIDATES = ("gcc", "cc", "clang")

BUILD_TIMEOUT_S = 180
RUN_TIMEOUT_S = 60


class BuildError(RuntimeError):
    """Собрать зонд не удалось (нет компилятора либо ошибка компиляции)."""


class ProbeError(RuntimeError):
    """Зонд собрался, но отработал не так, как обещает его протокол."""


# ── Экранирование (зеркало unescape()/print_escaped() в cmd_parser_probe.c) ──
_SIMPLE = {"\\": "\\\\", '"': '\\"', "\n": "\\n", "\r": "\\r", "\t": "\\t"}
_SIMPLE_BACK = {"\\": "\\", '"': '"', "n": "\n", "r": "\r", "t": "\t"}


def escape(text: str) -> str:
    """Строка в однострочном виде: пробелы по краям и табуляция не теряются."""
    out = []
    for ch in text:
        if ch in _SIMPLE:
            out.append(_SIMPLE[ch])
        elif ord(ch) < 0x20 or ord(ch) == 0x7F:
            out.append(f"\\x{ord(ch):02x}")
        else:
            out.append(ch)
    return "".join(out)


def unescape(text: str) -> str:
    """Обратное преобразование — им же сверяется эхо зонда с тем, что послали."""
    out = []
    i = 0
    while i < len(text):
        ch = text[i]
        if ch != "\\":
            out.append(ch)
            i += 1
            continue
        if i + 1 >= len(text):
            raise ProbeError("оборванная escape-последовательность: " + text)
        nxt = text[i + 1]
        if nxt in _SIMPLE_BACK:
            out.append(_SIMPLE_BACK[nxt])
            i += 2
        elif nxt == "x":
            out.append(chr(int(text[i + 2:i + 4], 16)))
            i += 4
        else:
            raise ProbeError("неизвестная escape-последовательность: " + text)
    return "".join(out)


# ── Результат разбора одной строки ──────────────────────────────────────────
class Result:
    """Что вернул Cmd_Parse() на одну строку.

    text   — сама строка (как её увидел парсер);
    rc     — возврат Cmd_Parse: 0 = прошивка промолчит, 1 = команда принята;
    type   — Cmd_Type при rc=1, иначе None (при rc=0 поля out прошивка не
             смотрит, поэтому зонд их и не печатает);
    fields — значимые поля Cmd_Result строками, как их напечатал зонд.
    """

    __slots__ = ("text", "rc", "type", "fields")

    def __init__(self, text: str, rc: int, type_: str | None, fields: dict):
        self.text = text
        self.rc = rc
        self.type = type_
        self.fields = fields

    @property
    def accepted(self) -> bool:
        """Парсер строку принял (прошивка ответит хоть что-нибудь)."""
        return self.rc == 1

    @property
    def unknown(self) -> bool:
        """Принята как «команда не распознана» — прошивка ответит err:unknown."""
        return self.rc == 1 and self.type == "CMD_UNKNOWN"

    @property
    def silent(self) -> bool:
        """Парсер строку отверг — прошивка на неё не отвечает вовсе."""
        return self.rc == 0

    def num(self, key: str) -> float:
        """Числовое поле результата (float; для целых значение точное)."""
        if key not in self.fields:
            raise ProbeError(f"в разборе '{self.text}' нет поля {key}: "
                             f"{self.fields}")
        return float(self.fields[key])

    def __str__(self) -> str:
        tail = "".join(f" {k}={v}" for k, v in self.fields.items())
        head = f'in="{escape(self.text)}" rc={self.rc}'
        return head if self.type is None else f"{head} type={self.type}{tail}"


_RE_OUT = re.compile(r'^in="((?:[^"\\]|\\.)*)" rc=(\d+)(.*)$')


def _parse_output_line(line: str) -> Result:
    m = _RE_OUT.match(line)
    if m is None:
        raise ProbeError("зонд напечатал строку не по своему протоколу: " + line)
    text = unescape(m.group(1))
    rc = int(m.group(2))
    rest = m.group(3).split()
    type_ = None
    fields: dict[str, str] = {}
    for i, tok in enumerate(rest):
        key, _, val = tok.partition("=")
        if i == 0:
            if key != "type":
                raise ProbeError("после rc ожидалось type=: " + line)
            type_ = val
        else:
            fields[key] = val
    if rc == 1 and type_ is None:
        raise ProbeError("rc=1 без type=: " + line)
    if rc == 0 and type_ is not None:
        raise ProbeError("rc=0, а type= напечатан: " + line)
    return Result(text, rc, type_, fields)


# ── Сборка и прогон ─────────────────────────────────────────────────────────
def find_cc(cc: str | None = None) -> str | None:
    """Путь к компилятору C или None. Переменная окружения CC — в приоритете."""
    for name in (cc, os.environ.get("CC")) + CC_CANDIDATES:
        if name:
            found = shutil.which(name)
            if found:
                return found
    return None


def build(out_dir: str, cc: str | None = None) -> str:
    """Собирает зонд в out_dir и возвращает путь к исполняемому файлу."""
    compiler = find_cc(cc)
    if compiler is None:
        raise BuildError(
            "компилятор C не найден: искали " +
            ", ".join(CC_CANDIDATES) + " в PATH и переменную окружения CC")
    for path in (SRC_PARSER, SRC_PROBE, INCLUDE_DIR):
        if not os.path.exists(path):
            raise BuildError("нет исходника зонда: " + path)

    os.makedirs(out_dir, exist_ok=True)
    exe = os.path.join(out_dir,
                       "cmd_parser_probe.exe" if os.name == "nt"
                       else "cmd_parser_probe")
    cmd = [compiler, *CFLAGS, "-I", INCLUDE_DIR,
           SRC_PARSER, SRC_PROBE, "-lm", "-o", exe]
    try:
        proc = subprocess.run(cmd, capture_output=True, timeout=BUILD_TIMEOUT_S)
    except OSError as exc:                      # компилятор есть, но не запустился
        raise BuildError(f"{compiler} не запустился: {exc}") from exc
    except subprocess.TimeoutExpired as exc:
        raise BuildError(f"сборка зонда не уложилась в {BUILD_TIMEOUT_S} с") from exc
    if proc.returncode != 0 or not os.path.exists(exe):
        raise BuildError(
            "сборка зонда не удалась (" + " ".join(cmd) + "):\n" +
            proc.stderr.decode("utf-8", "replace") +
            proc.stdout.decode("utf-8", "replace"))
    return exe


def run(exe: str, lines) -> list[Result]:
    """Прогоняет строки через зонд. Порядок результатов — порядок входа."""
    lines = list(lines)
    for text in lines:
        if "\n" in text or "\r" in text:
            # Терминатор до Cmd_Parse не доходит: его снимает сборщик строки
            # (line_reader.c). Строка с ним внутри — ошибка теста, а не случай
            # протокола, и делать вид, что она проверена, нельзя.
            raise ProbeError("в проверяемой строке CR/LF: " + repr(text))
    payload = "".join(escape(t) + "\n" for t in lines).encode("utf-8")
    try:
        proc = subprocess.run([exe], input=payload, capture_output=True,
                              timeout=RUN_TIMEOUT_S)
    except subprocess.TimeoutExpired as exc:
        raise ProbeError(f"зонд не ответил за {RUN_TIMEOUT_S} с") from exc
    err = proc.stderr.decode("utf-8", "replace").strip()
    if proc.returncode != 0:
        raise ProbeError(f"зонд вышел с кодом {proc.returncode}: {err}")
    out = proc.stdout.decode("utf-8", "replace").splitlines()
    if len(out) != len(lines):
        raise ProbeError(f"зонд вернул {len(out)} строк на {len(lines)} входных")
    results = [_parse_output_line(line) for line in out]
    for text, res in zip(lines, results):
        if res.text != text:
            # Эхо не совпало — значит строки разъехались, и сверять результаты
            # уже не с чем.
            raise ProbeError(f"эхо зонда {res.text!r} вместо {text!r}")
    return results


def probe(lines, out_dir: str | None = None, cc: str | None = None) -> list[Result]:
    """Собрать и прогнать за один вызов (сборка во временном каталоге)."""
    if out_dir is not None:
        return run(build(out_dir, cc), lines)
    tmp = tempfile.mkdtemp(prefix="cmd_parser_probe-")
    try:
        return run(build(tmp, cc), lines)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ── Показательный корпус ────────────────────────────────────────────────────
# Строки, на которых видно и норму, и края протокола. Юнит-тест
# SonarDebugGUI (tests/test_cmd_parser_host.py) берёт этот же список и
# добавляет к нему то, что умеет собрать сам (вывод построителей protocol.py и
# пробы границ), — чтобы «интересные строки» жили в одном месте.
CORPUS = (
    # штатные команды
    "en", "dis", "stop", "mcfg", "diag",
    "t=90", "t=-30", "t=370", "t=+", "t=-",
    "kp=0.025", "ki=0.5", "kd=0.1", "v=600", "a=2000",
    "op=20", "om=0", "om=1", "om=2", "debug=1",
    "scan=0,90,5,100", "scan=0,+,5,100", "scan=350,10,5,100",
    "sync=0", "sync=2", "hold=0", "hold=1",
    "irun 800", "ihold 300", "icur 700 200", "mstep 16",
    # запросы состояния без аргумента — только точное совпадение
    "sync", "om", "hold",
    # края: аргумент есть, значения нет
    "t=", "hold=", "om=", "op=", "sync=", "debug=", "scan=", "mstep ", "irun ",
    # края: похоже на команду, но не она
    "omx", "omm", "OM", " om", "om ", "holdx", "syncx", "en ", "ens",
    "icur", "icur 700", "mstep", "t=++",
    # края: границы числовых аргументов
    "om=3", "om=-1", "sync=3", "hold=2", "op=65535", "op=65536",
    "mstep 0", "mstep 3", "mstep 256", "mstep 65535", "mstep 65536",
    "mstep 65537", "irun 3000", "irun 3001", "ihold 3001",
    # края: форма числа
    "t=0x10", "t=1e40", "t=inf", "t=nan", "op=0x10", "om= 1", "icur 700\t200",
    # края: пустая строка, одни пробелы, очень длинная строка
    "", "   ", "\t", "x" * 300, "t=" + "9" * 300,
)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Сборка и прогон хостового зонда парсера команд прошивки")
    ap.add_argument("--stdin", action="store_true",
                    help="брать проверяемые строки из stdin (по строке на вход, "
                         "обратные слэши раскрываются), а не из корпуса")
    ap.add_argument("--out", metavar="DIR",
                    help="каталог сборки (по умолчанию временный, удаляется)")
    ap.add_argument("--cc", help="компилятор C (по умолчанию $CC, затем gcc/cc/clang)")
    args = ap.parse_args(argv)

    # В стандартной консоли Windows (cp866/cp1251) первое же русское сообщение
    # уронило бы прогон с UnicodeEncodeError вместо того, чтобы показать причину.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError, OSError):
            pass        # перенаправленный в файл/конвейер поток трогать не обязательно

    if args.stdin:
        lines = [unescape(s.rstrip("\r\n")) for s in sys.stdin.read().splitlines()]
    else:
        lines = list(CORPUS)

    try:
        results = probe(lines, out_dir=args.out, cc=args.cc)
    except (BuildError, ProbeError) as exc:
        print(f"ОШИБКА: {exc}", file=sys.stderr)
        return 2

    for res in results:
        print(res)
    print(f"\nразобрано строк: {len(results)}; принято парсером: "
          f"{sum(r.accepted for r in results)}; "
          f"из них не распознано: {sum(r.unknown for r in results)}; "
          f"отвергнуто (прошивка промолчит): {sum(r.silent for r in results)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
