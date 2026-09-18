#!/usr/bin/env python3
"""
Тест всех команд FW_SonarMotorDriver с логом TX/RX и привязкой по времени.
Покрывает: en, dis, stop, t=X/t=+/t=-, kp/ki/kd, op, debug, scan, irun, ihold,
icur, mstep, mcfg, установки sync=/om=/hold= и запросы состояния sync/om/hold,
а также отрицательные случаи (значение вне диапазона прошивка отвергает молча —
см. check_silent).

НЕ ПРОВЕРЕНО НА ПЛАТЕ. Сценарии sync=/sync, om=/om и hold=/hold дописаны вместе
с самими командами и на реальном железе ни разу не прогонялись: скрипту нужен
физический COM-порт, платы не было. Проверено только чтением и
`python -m py_compile`.
Те же команды гоняются против Python-модели прошивки
(Software/SonarDebugGUI/protocol_test.py --sim), но модель — не плата: реальных
таймингов UART, фронта на SYNC_IN и ответов TMC2209 на irun/mstep в ней нет.
Первый прогон на железе считать отладочным: расхождение вероятнее в ожиданиях
этого скрипта, чем в прошивке.
"""

import sys
import argparse
import re
import time
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass
from typing import Callable

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Нужен pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

BAUD = 115200
LOG_DIR = Path(__file__).resolve().parent.parent / "logs"
RX_TIMEOUT = 0.5
IDLE_TIMEOUT_MS = 100  # завершить чтение если нет данных N мс после хотя бы одной строки

# Полный зигзаг 0..180° шаг 10° задержка 100 мс: 37 позиций. Цикл одной точки —
# ход плюс пауза; ход задаёт ПИД (при kp=0.025 предел v= на шаге 10° не
# работает вовсе): ln(10/0.05)/25 ≈ 0.21 с + 0.1 с паузы ≈ 0.31 с на точку,
# то есть ≈ 11.5 с на весь зигзаг. Ждём 30 с — с запасом на медленные kp.
SCAN_FULL_DURATION_SEC = 30


def _relax_console_encoding() -> None:
    """
    Консоль Windows живёт в cp866/cp1251, а в названиях тестов есть символы
    вроде «→». Без этого печать такого названия валит прогон UnicodeEncodeError
    посреди теста. В файл лога (utf-8) всё пишется как есть, подменяются только
    непечатаемые в консоли символы.
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, OSError, ValueError):
            pass


_relax_console_encoding()


def _join_lines(lines: list[str]) -> str:
    return "\n".join(lines)


# --- Проверки (checker) возвращают (ok: bool, fail_reason: str) ---

def check_ok(lines: list[str]) -> tuple[bool, str]:
    """Любой ответ с ok: считается успехом."""
    text = _join_lines(lines)
    if "ok:" in text:
        return True, ""
    if "err:" in text:
        return False, f"ожидалось ok:, получено err: — {text[:100]}"
    return False, f"нет ok: в ответе: {lines[:5]!r}"


def check_ok_prefix(prefix: str) -> Callable[[list[str]], tuple[bool, str]]:
    """Проверяет наличие ok:<prefix> в ответе."""
    def _check(lines: list[str]) -> tuple[bool, str]:
        text = _join_lines(lines)
        if f"ok:{prefix}" in text or f"ok: {prefix}" in text:
            return True, ""
        if "err:" in text:
            return False, f"ожидалось ok:{prefix}, получено err: — {text[:100]}"
        return False, f"нет ok:{prefix} в ответе: {text[:100]}"
    return _check


def check_mcfg(lines: list[str]) -> tuple[bool, str]:
    """mcfg возвращает mode=... run=... hold=... microsteps=... ready=..."""
    text = _join_lines(lines)
    if "mode=" in text and ("run=" in text or "hold=" in text):
        return True, ""
    if "err:" in text:
        return False, f"mcfg вернул err: — {text[:100]}"
    return False, f"неполный ответ mcfg: {text[:100]}"


def check_telemetry(lines: list[str]) -> tuple[bool, str]:
    """Телеметрия: cp=... ec=... (debug=0) или полная (debug=1)."""
    text = _join_lines(lines)
    if "cp:" in text or "ec:" in text:
        return True, ""
    return False, f"нет телеметрии cp/ec: {text[:100]}"


def check_ev_frame(lines: list[str]) -> tuple[bool, str]:
    """Режимы om=1/2: после команды движения должен прийти кадр с полем ev:1."""
    text = _join_lines(lines)
    if ",ev:1" in text:
        return True, ""
    if "err:stall" in text:
        return False, f"вал не дошёл до цели (err:stall): {text[:100]}"
    return False, f"нет кадра ev:1 после прихода в цель: {text[:120]}"


def check_err_unknown(lines: list[str]) -> tuple[bool, str]:
    """Ожидаем err:unknown для неизвестной команды."""
    text = _join_lines(lines)
    if "err:unknown" in text:
        return True, ""
    return False, f"ожидалось err:unknown: {text[:80]}"


def check_no_ev(lines: list[str]) -> tuple[bool, str]:
    """
    Режим om=0: телеметрия идёт только по таймеру op, событийного кадра при
    приходе в цель быть не должно (в прошивке приход в цель в этом режиме
    не взводит событие). Заодно проверяем, что периодический поток жив.
    """
    text = _join_lines(lines)
    if ",ev:1" in text:
        return False, f"в режиме om=0 пришёл событийный кадр ev:1: {text[:120]}"
    if "cp:" not in text:
        return False, f"нет периодической телеметрии cp:: {text[:120]}"
    return True, ""


def check_silent(lines: list[str]) -> tuple[bool, str]:
    """
    Аргумент вне допустимого диапазона (sync=3, om=3, hold=2, нечисловой …):
    парсер отвергает строку целиком, обработчик команд даже не вызывается —
    прошивка не отвечает ничем. Это не то же самое, что err:unknown у
    нераспознанного слова. Кадры телеметрии (cp:/ec:) молчанием не считаются.
    """
    replies = [ln for ln in lines if "ok:" in ln or "err:" in ln]
    if replies:
        return False, f"ожидалось молчание парсера, получено: {replies[:3]!r}"
    return True, ""


# Ответ на запрос состояния синхронизации: "sync=<0..2> in=<0/1> out=<0/1> n=<счётчик>"
_SYNC_STATE_RE = re.compile(r"\bsync=(\d+)\s+in=([01])\s+out=([01])\s+n=(\d+)\b")

# Счётчик фронтов SYNC_IN из предыдущего запроса sync (см. check_sync_state)
_sync_prev_n: int | None = None


def check_sync_state(expected_mode: int) -> Callable[[list[str]], tuple[bool, str]]:
    """
    Запрос sync. Уровни in=/out= задаёт внешний жгут, поэтому сверяем только
    формат (0/1); режим обязан совпасть с только что установленным — это
    проверка, что sync= действительно сохранён, а отвергнутое значение нет.
    Счётчик n в прошивке только инкрементируется и нигде не сбрасывается,
    поэтому между двумя запросами он не может уменьшиться.
    """
    def _check(lines: list[str]) -> tuple[bool, str]:
        global _sync_prev_n
        text = _join_lines(lines)
        m = _SYNC_STATE_RE.search(text)
        if not m:
            return False, f"нет ответа вида 'sync=N in=0/1 out=0/1 n=K': {text[:120]}"
        mode = int(m.group(1))
        edges = int(m.group(4))
        if mode != expected_mode:
            return False, f"ожидался sync={expected_mode}, прошивка сообщает sync={mode}"
        if _sync_prev_n is not None and edges < _sync_prev_n:
            return False, (f"счётчик фронтов SYNC_IN уменьшился: "
                           f"было n={_sync_prev_n}, стало n={edges}")
        _sync_prev_n = edges
        return True, ""
    return _check


def check_state_query(keyword: str, expected: int) -> Callable[[list[str]], tuple[bool, str]]:
    """
    Запрос состояния `om` или `hold` — ключевое слово без `=`. Парсер узнаёт
    его сравнением строки целиком (cmd_parser.c:249 `strcmp(line, "om")` и
    cmd_parser.c:256 `strcmp(line, "hold")`), а обработчик отвечает ОДНОЙ
    строкой без префикса `ok:` — `om=<0..2>` (main.c:1648) и `hold=<0|1>`
    (main.c:1644), как у запроса `sync`.

    Проверяем три вещи сразу:
      * ответ вообще есть и имеет вид `<keyword>=N` целой строкой;
      * значение совпадает с только что установленным — то есть плата хранит
        режим, а не пересказывает последнюю команду;
      * префикса `ok:` в ответе НЕТ. Без этой проверки эхо установки
        (`ok:om=1`) сошло бы за ответ на запрос, и подмена разбора осталась бы
        незамеченной.
    Кадры телеметрии (cp:/ec:) под шаблон целой строки не подходят и не мешают.
    """
    pattern = re.compile(rf"^{keyword}=(\d+)$")

    def _check(lines: list[str]) -> tuple[bool, str]:
        echoes = [ln for ln in lines if ln.startswith("ok:")]
        if echoes:
            return False, (f"на запрос '{keyword}' пришло подтверждение установки "
                           f"{echoes[0]!r}: строка разобрана не как запрос")
        for ln in lines:
            m = pattern.match(ln)
            if m:
                value = int(m.group(1))
                if value != expected:
                    return False, (f"ожидалось {keyword}={expected}, "
                                   f"прошивка сообщает {keyword}={value}")
                return True, ""
        text = _join_lines(lines)
        return False, f"нет ответа вида '{keyword}=N' без ok:: {text[:120]}"
    return _check


@dataclass
class TestCase:
    cmd: str
    name: str
    checker: Callable[[list[str]], tuple[bool, str]]
    wait_extra_ms: int = 0
    delay_after_ms: int = 0   # пауза после команды (для scan, t=±)
    sleep_after_sec: float = 0  # ожидание после ответа (для полного теста сканирования)
    # Предел чтения ответа. Отсчёт до идущих подряд кадров телеметрии не
    # завершается досрочно, поэтому обычный шаг занимает весь предел. Там, где
    # ответа не ждут вовсе (отвергнутый аргумент), хватает доли секунды.
    read_timeout_sec: float = 3.0


class Logger:
    """Логгер с выводом в файл и консоль, привязка по времени."""

    def __init__(self, log_path: Path):
        self.log_path = log_path
        self.f = open(log_path, "w", encoding="utf-8")
        self.f.write(f"# FW_SonarMotorDriver command test log — {datetime.now().isoformat()}\n")
        self.f.write("# [HH:MM:SS.mmm] TX = отправлено, RX = получено\n\n")
        self.f.flush()

    def log_tx(self, data: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] TX: {repr(data)}\n"
        self.f.write(line)
        self.f.flush()
        print(f"  TX: {data!r}")

    def log_rx(self, data: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] RX: {data}\n"
        self.f.write(line)
        self.f.flush()
        print(f"  RX: {data}")

    def log_info(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] --- {msg}\n"
        self.f.write(line)
        self.f.flush()
        print(f"  --- {msg}")

    def close(self):
        self.f.close()


def send_cmd(ser: serial.Serial, cmd: str, logger: Logger,
             idle_ms: float = IDLE_TIMEOUT_MS / 1000,
             wait_extra_ms: int = 0,
             read_timeout_sec: float = 3.0) -> list[str]:
    """
    Отправляет команду и возвращает список строк ответа.
    Использует idle timeout: завершает чтение, если после хотя бы одной строки
    линия молчит idle_ms секунд.
    """
    ser.reset_input_buffer()

    raw = cmd if cmd.endswith("\r\n") else cmd + "\r\n"
    logger.log_tx(raw.strip())
    ser.write(raw.encode("utf-8"))

    lines: list[str] = []
    buf = b""
    last_data_time = time.monotonic()
    t_end = last_data_time + read_timeout_sec

    while time.monotonic() < t_end:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n)
            buf += chunk
            last_data_time = time.monotonic()

        while True:
            pos_cr = buf.find(b"\r")
            pos_lf = buf.find(b"\n")
            if pos_cr == -1 and pos_lf == -1:
                break
            if pos_cr >= 0 and (pos_lf < 0 or pos_cr <= pos_lf):
                sep = b"\r"
            else:
                sep = b"\n"
            part, _, buf = buf.partition(sep)
            if part:
                try:
                    text = part.decode("utf-8", errors="replace").strip()
                except Exception:
                    text = ""
                if text:
                    lines.append(text)
                    logger.log_rx(text)

        if lines and not ser.in_waiting:
            elapsed = time.monotonic() - last_data_time
            if elapsed >= idle_ms:
                break

        if wait_extra_ms and lines:
            time.sleep(wait_extra_ms / 1000.0)
            if ser.in_waiting:
                continue
            break

        time.sleep(0.005)

    return lines


def find_stm32_port() -> str | None:
    """Пытается найти COM-порт STM32."""
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "0483" in hwid or "stm" in desc or "stm32" in desc or "ch340" in hwid or "cp210" in hwid:
            return p.device
    return None


def run_tests(ser: serial.Serial, logger: Logger, full_scan: bool = True) -> list[tuple[str, str, bool, str, list[str]]]:
    """Выполняет тесты. Возвращает список (name, cmd, ok, fail_reason, lines)."""
    test_cases: list[TestCase] = [
        # Базовые команды
        TestCase("dis", "disable", check_ok),
        TestCase("en", "enable", check_ok_prefix("en")),
        TestCase("t=90", "target 90°", check_ok_prefix("t="), wait_extra_ms=150),
        TestCase("t=+", "continuous +", check_ok_prefix("t=+")),
        TestCase("stop", "stop", check_ok_prefix("stop"), delay_after_ms=100),
        TestCase("t=-", "continuous -", check_ok_prefix("t=-")),
        TestCase("stop", "stop after t=-", check_ok_prefix("stop")),
        TestCase("t=0", "target 0°", check_ok_prefix("t=")),
        # PID: рекомендуемые из прошивки (board.h). Сначала Kp в 2 раза меньше, затем исходный
        TestCase("kp=0.0125", "kp половинный (0.0125)", check_ok_prefix("kp=")),
        TestCase("ki=0", "ki=0", check_ok_prefix("ki=")),
        TestCase("kd=0", "kd=0", check_ok_prefix("kd=")),
        TestCase("kp=0.025", "kp исходный (0.025)", check_ok_prefix("kp=")),
        # Телеметрия (рекомендуемые: op=4, debug=0)
        TestCase("op=0", "op=0 (выкл телеметрия)", check_ok_prefix("op=")),
        # Частота потока = 1/op= только при debug=0: при debug=1 прошивка
        # поднимает период до OUTPUT_PERIOD_MS_DEBUG_MIN = 20 мс (main.c:1213).
        TestCase("op=4", "op=4 (период 4 мс, 250 Гц при debug=0)", check_ok_prefix("op=")),
        TestCase("debug=0", "debug=0", check_ok_prefix("debug=")),
        TestCase("debug=1", "debug=1", check_ok_prefix("debug=")),
        # Режим выдачи телеметрии: 1 = по достижению цели (кадр с ev:1), 2 = оба, 0 = штатный
        TestCase("om=1", "om=1 (по достижению позиции)", check_ok_prefix("om=")),
        # Запрос без аргумента: ответ «om=1» без ok:. Нужен хосту после
        # переподключения — своё последнее отправленное значение он мог
        # потерять, да и плата могла перезагрузиться. В телеметрии поля нет.
        TestCase("om", "om — запрос режима, ответ без ok:, режим 1", check_state_query("om", 1)),
        TestCase("t=45", "target 45° → кадр ev:1", check_ev_frame, wait_extra_ms=800),
        TestCase("om=2", "om=2 (таймер + достижение)", check_ok_prefix("om=")),
        TestCase("om", "om — режим 2 сохранён", check_state_query("om", 2)),
        TestCase("t=0", "target 0° → кадр ev:1 среди периодических", check_ev_frame, wait_extra_ms=800),
        TestCase("om=0", "om=0 (только таймер)", check_ok_prefix("om=0")),
        TestCase("om", "om — режим 0 сохранён", check_state_query("om", 0)),
        TestCase("t=90", "om=0: цель 90° — кадра ev:1 быть не должно", check_no_ev, wait_extra_ms=800),
        # Сканирование: полный тест зигзага 0..180°, шаг 10°, задержка 100 мс (ожидание SCAN_FULL_DURATION_SEC)
        TestCase("scan=0,180,10,100", "scan zigzag full 0..180° step 10° delay 100ms", check_ok, wait_extra_ms=500, sleep_after_sec=SCAN_FULL_DURATION_SEC),
        TestCase("stop", "stop scan", check_ok_prefix("stop")),
        # Краткая проверка бесконечного сканирования (сразу stop)
        TestCase("scan=0,+,10,100", "scan infinite +", check_ok, wait_extra_ms=200),
        TestCase("stop", "stop scan+", check_ok_prefix("stop")),
        TestCase("scan=0,-,10,100", "scan infinite -", check_ok, wait_extra_ms=200),
        TestCase("stop", "stop scan-", check_ok_prefix("stop")),
        # Синхронизация скана: источник перехода к следующей точке. Блок идёт
        # после сканов и завершается возвратом sync=0, иначе следующий скан
        # ждал бы фронт SYNC_IN, которого на стенде без генератора не будет.
        # Само поведение sync=1/2 (переход по фронту) проверяется только с
        # внешним источником импульсов — здесь проверяем приём команды,
        # хранение режима и ответ на запрос состояния.
        TestCase("sync=0", "sync=0 (переход по таймеру delay)", check_ok_prefix("sync=0")),
        TestCase("sync", "sync — запрос состояния, режим 0", check_sync_state(0)),
        TestCase("sync=1", "sync=1 (переход по фронту SYNC_IN)", check_ok_prefix("sync=1")),
        TestCase("sync", "sync — режим 1 сохранён", check_sync_state(1)),
        TestCase("sync=2", "sync=2 (фронт SYNC_IN или delay как тайм-аут)", check_ok_prefix("sync=2")),
        TestCase("sync", "sync — режим 2 сохранён", check_sync_state(2)),
        TestCase("sync=0", "sync=0 (вернуть таймер перед остальными тестами)", check_ok_prefix("sync=0")),
        # Удержание вала: hold=0 обесточивает обмотки на время замера, hold=1
        # возвращает ток — цель при этом сохраняется (в отличие от dis/en)
        TestCase("t=30", "target 30° перед снятием удержания", check_ok_prefix("t="), wait_extra_ms=400),
        TestCase("hold=0", "hold=0 (снять удержание)", check_ok_prefix("hold=0")),
        # Запрос без аргумента: ответ «hold=0» без ok:. Здесь это ещё и вопрос
        # безопасности — обесточенный вал с виду не отличается от стоящего
        # под током, а в телеметрии этого поля тоже нет.
        TestCase("hold", "hold — запрос удержания, ответ без ok:, снято", check_state_query("hold", 0)),
        # При снятом удержании команды принимаются и запоминаются, телеметрия
        # продолжает идти (контур на паузе, выдача кадров от hold не зависит)
        TestCase("t=30", "t= при hold=0 принимается", check_ok_prefix("t=30")),
        TestCase("sync", "sync при hold=0 отвечает, режим 0", check_sync_state(0)),
        TestCase("hold=1", "hold=1 (вернуть удержание)", check_ok_prefix("hold=1"), wait_extra_ms=300),
        TestCase("hold", "hold — удержание вернулось", check_state_query("hold", 1)),
        # Ток и микрошаг (рекомендуемые из прошивки: 600, 300, 256)
        TestCase("irun 600", "irun 600", check_ok_prefix("irun=")),
        TestCase("ihold 300", "ihold 300", check_ok_prefix("ihold=")),
        TestCase("icur 600 300", "icur 600 300", check_ok_prefix("icur=")),
        TestCase("mstep 256", "mstep 256", check_ok_prefix("mstep=")),
        # Конфигурация
        TestCase("mcfg", "mcfg", check_mcfg, wait_extra_ms=100),
        # Неизвестная команда
        TestCase("xyz123", "unknown cmd → err:unknown", check_err_unknown),
        # Отрицательные случаи. Нераспознанное слово даёт err:unknown, а вот
        # распознанная команда с негодным аргументом отвергается ещё парсером —
        # обработчик не вызывается, и прошивка не отвечает ничем. Телеметрию на
        # время блока глушим (op=0), чтобы молчание проверялось по чистой линии;
        # ответа здесь не ждут, поэтому предел чтения короткий.
        TestCase("op=0", "op=0 (тишина для отрицательных проверок)", check_ok_prefix("op=0")),
        TestCase("sync=3", "sync=3 вне 0..2 → молчание", check_silent, read_timeout_sec=0.5),
        TestCase("sync=-1", "sync=-1 вне 0..2 → молчание", check_silent, read_timeout_sec=0.5),
        TestCase("sync=x", "sync=x не число → молчание", check_silent, read_timeout_sec=0.5),
        TestCase("syncx", "syncx — не команда → err:unknown", check_err_unknown, read_timeout_sec=0.5),
        TestCase("om=3", "om=3 вне 0..2 → молчание", check_silent, read_timeout_sec=0.5),
        TestCase("om=-1", "om=-1 вне 0..2 → молчание", check_silent, read_timeout_sec=0.5),
        TestCase("om=", "om= без значения → молчание", check_silent, read_timeout_sec=0.5),
        # Запросы состояния узнаются только ЦЕЛИКОМ: лишняя буква обязана дать
        # err:unknown, а не «om» с проглоченным хвостом. Проверка того же
        # свойства, что и syncx выше.
        TestCase("omx", "omx — не команда → err:unknown", check_err_unknown, read_timeout_sec=0.5),
        TestCase("hold=2", "hold=2 вне 0..1 → молчание", check_silent, read_timeout_sec=0.5),
        TestCase("hold=-1", "hold=-1 вне 0..1 → молчание", check_silent, read_timeout_sec=0.5),
        TestCase("hold=y", "hold=y не число → молчание", check_silent, read_timeout_sec=0.5),
        TestCase("holdx", "holdx — не команда → err:unknown", check_err_unknown, read_timeout_sec=0.5),
        # Отвергнутые значения не должны были ничего изменить
        TestCase("sync", "sync после отказов: режим остался 0", check_sync_state(0), read_timeout_sec=0.5),
        TestCase("om", "om после отказов: режим остался 0", check_state_query("om", 0), read_timeout_sec=0.5),
        TestCase("hold", "hold после отказов: удержание осталось 1", check_state_query("hold", 1), read_timeout_sec=0.5),
        # debug=1 здесь всё ещё включён (выше по сценарию), поэтому поток
        # пойдёт не на 250 Гц: период поднимется до OUTPUT_PERIOD_MS_DEBUG_MIN
        # = 20 мс, то есть 50 Гц (main.c:1213).
        TestCase("op=4", "op=4 (вернуть телеметрию: 4 мс, при debug=1 это 20 мс / 50 Гц)", check_ok_prefix("op=4")),
        # Финал
        TestCase("dis", "disable final", check_ok),
        # hold=1 при выключенном приводе только запоминает флаг: ток появится
        # по команде en, вал остаётся свободным
        TestCase("hold=1", "hold=1 после dis — флаг принят", check_ok_prefix("hold=1")),
        # g_hold присваивается независимо от того, включён ли привод
        # (main.c:1611), поэтому запрос обязан сообщить 1 и при dis.
        TestCase("hold", "hold после dis — запрос сообщает 1", check_state_query("hold", 1)),
    ]

    results: list[tuple[str, str, bool, str, list[str]]] = []

    for tc in test_cases:
        logger.log_info(f"TEST: {tc.name} (cmd='{tc.cmd}')")
        try:
            lines = send_cmd(ser, tc.cmd, logger, wait_extra_ms=tc.wait_extra_ms,
                             read_timeout_sec=tc.read_timeout_sec)
            ok, reason = tc.checker(lines)
            results.append((tc.name, tc.cmd, ok, reason, lines))
            if ok:
                logger.log_info("  -> PASS")
            else:
                logger.log_info(f"  -> FAIL: {reason}")
            if tc.delay_after_ms:
                time.sleep(tc.delay_after_ms / 1000.0)
            sleep_sec = tc.sleep_after_sec
            if not full_scan and sleep_sec == SCAN_FULL_DURATION_SEC:
                sleep_sec = 5.0
            if sleep_sec > 0:
                logger.log_info(f"  ожидание полного сканирования {sleep_sec:.0f} с...")
                time.sleep(sleep_sec)
                logger.log_info(f"  ожидание завершено")
        except Exception as e:
            results.append((tc.name, tc.cmd, False, str(e), []))
            logger.log_info(f"  -> ERROR: {e}")

    return results


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Тест всех команд FW_SonarMotorDriver с логом TX/RX (привязка по времени)"
    )
    parser.add_argument("port", nargs="?", default="COM19", help="COM-порт (по умолчанию COM19)")
    parser.add_argument("--no-test", action="store_true", help="Только лог, без автотеста")
    parser.add_argument("--no-full-scan", action="store_true", help="Не ждать полный зигзаг сканирования (~30 с), только 5 с")
    parser.add_argument("--log", "-l", help="Путь к лог-файлу")
    args = parser.parse_args()

    port = args.port.strip() if args.port else ""
    if not port:
        found = find_stm32_port()
        port = found or ""

    if not port:
        print("Порт не указан и автоопределение не сработало.", file=sys.stderr)
        print("\nДоступные порты:", file=sys.stderr)
        for p in list_ports.comports():
            print(f"  {p.device} — {p.description or '?'} ({p.hwid or '?'})", file=sys.stderr)
        print("\nУкажите порт: python test_commands.py COM3", file=sys.stderr)
        return 1

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = Path(args.log) if args.log else LOG_DIR / f"fw_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    logger = Logger(log_path)

    print(f"Порт: {port}")
    print(f"Скорость: {BAUD}")
    print(f"Лог: {log_path}\n")

    try:
        ser = serial.Serial(port, BAUD, timeout=RX_TIMEOUT)
    except serial.SerialException as e:
        print(f"Ошибка: {e}", file=sys.stderr)
        logger.log_info(f"ERROR: {e}")
        logger.close()
        return 1

    exit_code = 0
    try:
        time.sleep(0.5)

        if args.no_test:
            logger.log_info("Режим лога (--no-test). Вводите команды, Ctrl+C — выход.")
            import threading
            stop_flag = [False]

            def read_serial():
                buf = b""
                while ser.is_open and not stop_flag[0]:
                    n = ser.in_waiting
                    if n:
                        chunk = ser.read(n)
                        buf += chunk
                        while b"\r" in buf or b"\n" in buf:
                            pos_cr = buf.find(b"\r")
                            pos_lf = buf.find(b"\n")
                            sep = b"\r" if (pos_cr >= 0 and (pos_lf < 0 or pos_cr <= pos_lf)) else b"\n"
                            part, _, buf = buf.partition(sep)
                            if part:
                                try:
                                    text = part.decode("utf-8", errors="replace").strip()
                                except Exception:
                                    text = ""
                                if text:
                                    logger.log_rx(text)
                                    print(text)
                    else:
                        time.sleep(0.01)

            t = threading.Thread(target=read_serial, daemon=True)
            t.start()
            try:
                while True:
                    line = sys.stdin.readline()
                    if not line:
                        break
                    line = line.rstrip("\r\n")
                    if line:
                        ser.reset_input_buffer()
                        logger.log_tx(line)
                        ser.write((line + "\r\n").encode("utf-8"))
            except KeyboardInterrupt:
                pass
            finally:
                stop_flag[0] = True
        else:
            results = run_tests(ser, logger, full_scan=not args.no_full_scan)
            passed = sum(1 for r in results if r[2])
            total = len(results)
            fails = [(r[0], r[3], r[4]) for r in results if not r[2]]

            logger.log_info("")
            logger.log_info("=" * 50)
            logger.log_info(f"Итого: {passed}/{total} тестов пройдено")
            for name, ok, reason, _ in [(r[0], r[2], r[3], r[4]) for r in results]:
                status = "PASS" if ok else f"FAIL ({reason})"
                logger.log_info(f"  {name}: {status}")
            logger.log_info("=" * 50)

            print(f"\n--- Результат: {passed}/{total} тестов ---")
            for name, ok, reason, _ in [(r[0], r[2], r[3], r[4]) for r in results]:
                sym = "[OK]" if ok else "[FAIL]"
                print(f"  {sym} {name}" + (f" — {reason}" if not ok else ""))

            if fails:
                exit_code = 1
                print("\nУпавшие тесты:")
                for name, reason, lines in fails:
                    print(f"  - {name}: {reason}")
                    if lines:
                        print(f"    Ответ: {lines[:5]}")

    except KeyboardInterrupt:
        logger.log_info("Прервано пользователем")
        exit_code = 130
    finally:
        ser.close()
        logger.close()

    print(f"\nЛог сохранён: {log_path}")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
