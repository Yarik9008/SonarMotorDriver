#!/usr/bin/env python3
"""
Тест всех команд Test_TMC2209 и лог TX/RX.
Надёжный автотест USB CDC: строгие проверки, автоопределение порта.
"""

import sys
import argparse
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
IDLE_TIMEOUT_MS = 80  # завершить чтение если нет данных N мс после хотя бы одной строки


def _normalize(s: str) -> str:
    return " ".join(s.split()).strip()


def _join_lines(lines: list[str]) -> str:
    return "\n".join(lines)


# --- Проверки (checker) возвращают (ok: bool, fail_reason: str) ---

def check_help(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    if "init" in text and "move" in text and "telemetry" in text:
        return True, ""
    return False, f"неполный help: {lines[:3]!r}"


def check_init(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    if "init ok" in text or "init OK" in text.lower():
        return True, ""
    if "init FAIL" in text or "init fail" in text.lower():
        return False, "init завершился с ошибкой"
    if "init" in text:
        return False, "ожидалось 'init ok', получено: " + text[:80]
    return False, f"нет ответа init: {lines!r}"


def check_version(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    if "transport error" in text:
        return False, "IOIN read failed (transport error)"
    if "IC=0x21 OK" in text or "IC=0x21 ok" in text.lower():
        return True, ""
    if "IC=" in text:
        return False, f"неверная версия (ожидается 0x21 OK): {text[:80]}"
    return False, f"нет IC= в ответе: {lines!r}"


def check_status(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    if "transport error" in text:
        return False, "DRV_STATUS read failed"
    if "DRV=0x" in text or "DRV=0X" in text:
        return True, ""
    return False, f"нет DRV= или transport error: {text[:80]}"


def check_telemetry(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    # Ключевые регистры не должны содержать "?"
    fail_regs = [r for r in ["GSTAT=?", "IFCNT=?", "IOIN=?", "DRV_STATUS=?"] if r in text]
    if fail_regs:
        return False, f"регистры не прочитаны: {', '.join(fail_regs)}"
    if "GSTAT=" in text and ("IFCNT=" in text or "IOIN=" in text):
        return True, ""
    return False, f"неполная телеметрия: {text[:120]}"


def check_on(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    if "on" in text.split():
        return True, ""
    return False, f"нет 'on': {lines!r}"


def check_off(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    if "off" in text.split():
        return True, ""
    return False, f"нет 'off': {lines!r}"


def check_move(expected: str) -> Callable[[list[str]], tuple[bool, str]]:
    def _check(ln: list[str]) -> tuple[bool, str]:
        t = _join_lines(ln)
        if f"={expected}" in t or f"UA={expected}" in t or f"SD={expected}" in t:
            return True, ""
        return False, f"ожидалось ={expected}: {ln!r}"
    return _check


def check_stop(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    if "stop" in text.split():
        return True, ""
    return False, f"нет 'stop': {lines!r}"


def check_mode_sd(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    if "mode:SD" in text:
        return True, ""
    return False, f"нет mode:SD: {lines!r}"


def check_mode_ua(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    if "mode:UA" in text:
        return True, ""
    return False, f"нет mode:UA: {lines!r}"


def check_cs_start(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    if "cs loop" in text:
        return True, ""
    return False, f"нет 'cs loop': {lines!r}"


def check_cs_stop(lines: list[str]) -> tuple[bool, str]:
    text = _join_lines(lines)
    if "stop" in text.split():
        return True, ""
    return False, f"ожидалось 'stop', получено: {lines!r}"


@dataclass
class TestCase:
    cmd: str
    name: str
    checker: Callable[[list[str]], tuple[bool, str]]
    wait_extra_ms: int = 0  # для команд с долгим выводом (t, h)


class Logger:
    """Логгер с выводом в файл и консоль."""

    def __init__(self, log_path: Path):
        self.log_path = log_path
        self.f = open(log_path, "w", encoding="utf-8")
        self.f.write(f"# Test_TMC2209 command log — {datetime.now().isoformat()}\n")
        self.f.write("# TX = отправлено, RX = получено\n\n")
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


def find_stm32_port() -> str | None:
    """Находит COM-порт STM32 (USB CDC)."""
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "0483" in hwid or "stm" in desc or "stm32" in desc or "cdc" in desc:
            return p.device
    return None


def send_cmd(ser: serial.Serial, cmd: str, logger: Logger, idle_ms: float = IDLE_TIMEOUT_MS / 1000,
             wait_extra_ms: int = 0) -> list[str]:
    """
    Отправляет команду и возвращает список строк ответа.
    Перед отправкой очищает входной буфер. Использует idle timeout:
    завершает чтение, если после хотя бы одной строки линия молчит idle_ms секунд.
    """
    ser.reset_input_buffer()

    raw = cmd if cmd.endswith("\r\n") else cmd + "\r\n"
    logger.log_tx(raw.strip())
    ser.write(raw.encode("utf-8"))

    lines: list[str] = []
    buf = b""
    last_data_time = time.monotonic()
    t_end = last_data_time + 3.0  # общий таймаут 3 сек

    while time.monotonic() < t_end:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n)
            buf += chunk
            last_data_time = time.monotonic()

        # Парсим строки (CR, LF, CRLF)
        while True:
            # Ищем первый разделитель
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

        # Idle timeout: если уже есть строки и линия молчит
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


def resolve_port(cli_port: str | None) -> tuple[str, bool]:
    """
    Определяет порт: CLI > авто > ошибка.
    Возвращает (port, auto_detected).
    """
    if cli_port:
        return cli_port.strip(), False
    found = find_stm32_port()
    if found:
        return found, True
    return "", False


def run_tests(ser: serial.Serial, logger: Logger) -> list[tuple[str, str, bool, str, list[str]]]:
    """
    Выполняет тесты. Возвращает список (name, cmd, ok, fail_reason, lines).
    """
    test_cases: list[TestCase] = [
        TestCase("h", "help", check_help),
        TestCase("i", "init", check_init, wait_extra_ms=500),
        TestCase("h", "help after init", check_help),
        TestCase("v", "version", check_version),
        TestCase("st", "status", check_status),
        TestCase("t", "telemetry", check_telemetry, wait_extra_ms=700),
        TestCase("e", "enable", check_on),
        TestCase("m100", "move 100", check_move("100")),
        TestCase("m0", "move 0", check_move("0")),
        TestCase("s", "stop", check_stop),
        TestCase("d", "disable", check_off),
        TestCase("e", "enable again", check_on),
        TestCase("p", "STEP/DIR mode", check_mode_sd),
        TestCase("u", "UART mode", check_mode_ua),
        TestCase("c", "cs loop start", check_cs_start),
    ]

    results: list[tuple[str, str, bool, str, list[str]]] = []

    for tc in test_cases:
        logger.log_info(f"TEST: {tc.name} (cmd='{tc.cmd}')")
        try:
            lines = send_cmd(ser, tc.cmd, logger, wait_extra_ms=tc.wait_extra_ms)
            ok, reason = tc.checker(lines)
            results.append((tc.name, tc.cmd, ok, reason, lines))
            if ok:
                logger.log_info("  -> PASS")
            else:
                logger.log_info(f"  -> FAIL: {reason}")
        except Exception as e:
            results.append((tc.name, tc.cmd, False, str(e), []))
            logger.log_info(f"  -> ERROR: {e}")

        if tc.cmd == "c":
            time.sleep(0.3)
            logger.log_info("TEST: cs loop stop (cmd='s')")
            try:
                lines = send_cmd(ser, "s", logger, idle_ms=0.15)
                ok, reason = check_cs_stop(lines)
                results.append(("cs loop stop", "s", ok, reason, lines))
                logger.log_info("  -> PASS" if ok else f"  -> FAIL: {reason}")
            except Exception as e:
                results.append(("cs loop stop", "s", False, str(e), []))
                logger.log_info(f"  -> ERROR: {e}")

    return results


def main() -> int:
    parser = argparse.ArgumentParser(description="Тест команд Test_TMC2209 с логом TX/RX")
    parser.add_argument("port", nargs="?", default="COM19", help="COM-порт (по умолчанию COM19)")
    parser.add_argument("--no-test", action="store_true", help="Только лог, без автотеста")
    parser.add_argument("--log", "-l", help="Путь к лог-файлу")
    args = parser.parse_args()

    port, auto_detected = resolve_port(args.port)
    if not port:
        print("Порт не указан и автоопределение не сработало.", file=sys.stderr)
        print("\nДоступные порты:", file=sys.stderr)
        for p in list_ports.comports():
            print(f"  {p.device} — {p.description or '?'} ({p.hwid or '?'})", file=sys.stderr)
        print("\nУкажите порт: python test_commands.py COM3", file=sys.stderr)
        return 1

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = Path(args.log) if args.log else LOG_DIR / f"tmc_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    logger = Logger(log_path)

    print(f"Порт: {port}" + (" (автоопределён)" if auto_detected else " (задан вручную)"))
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
            logger.log_info("Вывод идёт только из reader thread, без дублирования.\n")
            import threading

            stop_flag = [False]  # list to allow closure mutation

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
            results = run_tests(ser, logger)
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
