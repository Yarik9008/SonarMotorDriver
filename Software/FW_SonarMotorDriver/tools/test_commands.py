#!/usr/bin/env python3
"""
Тест всех команд FW_SonarMotorDriver с логом TX/RX и привязкой по времени.
Покрывает: en, dis, stop, t=X/t=+/t=-, kp/ki/kd, op, debug, scan, irun, ihold, icur, mstep, mcfg.
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
IDLE_TIMEOUT_MS = 100  # завершить чтение если нет данных N мс после хотя бы одной строки

# Полный зигзаг 0..180° шаг 10° задержка 100 мс: 37 позиций × 100 мс ≈ 3.7 с + время движения ≈ 30 с
SCAN_FULL_DURATION_SEC = 30


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


def check_err_unknown(lines: list[str]) -> tuple[bool, str]:
    """Ожидаем err:unknown для неизвестной команды."""
    text = _join_lines(lines)
    if "err:unknown" in text:
        return True, ""
    return False, f"ожидалось err:unknown: {text[:80]}"


@dataclass
class TestCase:
    cmd: str
    name: str
    checker: Callable[[list[str]], tuple[bool, str]]
    wait_extra_ms: int = 0
    delay_after_ms: int = 0   # пауза после команды (для scan, t=±)
    sleep_after_sec: float = 0  # ожидание после ответа (для полного теста сканирования)


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
             wait_extra_ms: int = 0) -> list[str]:
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
    t_end = last_data_time + 3.0

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
        TestCase("op=4", "op=4 (250 Гц)", check_ok_prefix("op=")),
        TestCase("debug=0", "debug=0", check_ok_prefix("debug=")),
        TestCase("debug=1", "debug=1", check_ok_prefix("debug=")),
        # Сканирование: полный тест зигзага 0..180°, шаг 10°, задержка 100 мс (ожидание SCAN_FULL_DURATION_SEC)
        TestCase("scan=0,180,10,100", "scan zigzag full 0..180° step 10° delay 100ms", check_ok, wait_extra_ms=500, sleep_after_sec=SCAN_FULL_DURATION_SEC),
        TestCase("stop", "stop scan", check_ok_prefix("stop")),
        # Краткая проверка бесконечного сканирования (сразу stop)
        TestCase("scan=0,+,10,100", "scan infinite +", check_ok, wait_extra_ms=200),
        TestCase("stop", "stop scan+", check_ok_prefix("stop")),
        TestCase("scan=0,-,10,100", "scan infinite -", check_ok, wait_extra_ms=200),
        TestCase("stop", "stop scan-", check_ok_prefix("stop")),
        # Ток и микрошаг (рекомендуемые из прошивки: 600, 300, 256)
        TestCase("irun 600", "irun 600", check_ok_prefix("irun=")),
        TestCase("ihold 300", "ihold 300", check_ok_prefix("ihold=")),
        TestCase("icur 600 300", "icur 600 300", check_ok_prefix("icur=")),
        TestCase("mstep 256", "mstep 256", check_ok_prefix("mstep=")),
        # Конфигурация
        TestCase("mcfg", "mcfg", check_mcfg, wait_extra_ms=100),
        # Неизвестная команда
        TestCase("xyz123", "unknown cmd → err:unknown", check_err_unknown),
        # Финал
        TestCase("dis", "disable final", check_ok),
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
