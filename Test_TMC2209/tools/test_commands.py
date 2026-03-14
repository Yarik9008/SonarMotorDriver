#!/usr/bin/env python3
"""
Тест всех команд Test_TMC2209 и лог TX/RX.
Логирует все отправленные и принятые данные в файл.
"""

import sys
import argparse
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Нужен pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

BAUD = 115200
DEFAULT_PORT = "COM24"
LOG_DIR = Path(__file__).resolve().parent.parent / "logs"
RX_TIMEOUT = 0.5
LINE_TIMEOUT = 2.0


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


def find_stm32_port():
    """Находит COM-порт STM32 (USB CDC)."""
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "0483" in hwid or "stm" in desc or "stm32" in desc or "cdc" in desc:
            return p.device
    return None


def send_cmd(ser: serial.Serial, cmd: str, logger: Logger) -> list[str]:
    """Отправляет команду и возвращает список строк ответа."""
    raw = cmd if cmd.endswith("\r\n") else cmd + "\r\n"
    logger.log_tx(raw.strip())
    ser.write(raw.encode("utf-8"))

    lines = []
    buf = b""
    t0 = datetime.now()
    while (datetime.now() - t0).total_seconds() < LINE_TIMEOUT:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        while b"\r" in buf or b"\n" in buf:
            sep = b"\r" if b"\r" in buf else b"\n"
            line, _, buf = buf.partition(sep)
            try:
                text = line.decode("utf-8", errors="replace").strip()
            except Exception:
                text = ""
            if text:
                lines.append(text)
                logger.log_rx(text)
            if not chunk and len(lines) > 0:
                return lines
        if lines and not ser.in_waiting:
            break
    return lines


def run_tests(ser: serial.Serial, logger: Logger) -> dict[str, bool]:
    """Выполняет тест всех команд. Возвращает {cmd: ok}."""
    results = {}

    tests = [
        ("h", "help", lambda l: any("init" in x for x in l)),
        ("i", "init", lambda l: "init" in "\n".join(l)),
        ("h", "help after init", lambda l: any("init" in x for x in l)),
        ("v", "version", lambda l: "IC=" in "\n".join(l)),
        ("st", "status", lambda l: "DRV=" in "\n".join(l)),
        ("t", "telemetry", lambda l: "GSTAT=" in "\n".join(l) or "GSTAT=?" in "\n".join(l)),
        ("e", "enable", lambda l: "on" in "\n".join(l)),
        ("m100", "move 100", lambda l: any("=100" in x for x in l)),
        ("m0", "move 0", lambda l: any("=0" in x for x in l)),
        ("s", "stop", lambda l: "stop" in "\n".join(l)),
        ("d", "disable", lambda l: "off" in "\n".join(l)),
        ("e", "enable again", lambda l: "on" in "\n".join(l)),
        ("p", "STEP/DIR mode", lambda l: "mode:SD" in "\n".join(l)),
        ("u", "UART mode", lambda l: "mode:UA" in "\n".join(l)),
        ("c", "cs loop start", lambda l: "cs loop" in "\n".join(l) or "cs:" in "\n".join(l)),
    ]

    for cmd, desc, check in tests:
        logger.log_info(f"TEST: {desc} (cmd={repr(cmd)})")
        try:
            lines = send_cmd(ser, cmd, logger)
            ok = check(lines)
            results[desc] = ok
            logger.log_info(f"  -> {'PASS' if ok else 'FAIL'}")
        except Exception as e:
            logger.log_info(f"  -> ERROR: {e}")
            results[desc] = False

        if cmd == "c":
            time.sleep(0.3)
            logger.log_info("TEST: cs loop stop (cmd='s')")
            lines = send_cmd(ser, "s", logger)
            results["cs loop stop"] = "stop" in "\n".join(lines)
            logger.log_info(f"  -> {'PASS' if results['cs loop stop'] else 'FAIL'}")

    return results


def main():
    parser = argparse.ArgumentParser(description="Тест команд Test_TMC2209 с логом TX/RX")
    parser.add_argument("port", nargs="?", help="COM-порт (например COM3)")
    parser.add_argument("--no-test", action="store_true", help="Только лог, без автотеста")
    parser.add_argument("--log", "-l", help="Путь к лог-файлу")
    args = parser.parse_args()

    port = args.port or DEFAULT_PORT
    if not port:
        print("Доступные порты:")
        for p in list_ports.comports():
            print(f"  {p.device} — {p.description} ({p.hwid})")
        print("\nУкажите порт: python test_commands.py COM3")
        sys.exit(1)

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = Path(args.log) if args.log else LOG_DIR / f"tmc_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    logger = Logger(log_path)

    print(f"Подключение к {port} @ {BAUD}")
    print(f"Лог: {log_path}\n")

    try:
        ser = serial.Serial(port, BAUD, timeout=RX_TIMEOUT)
    except serial.SerialException as e:
        print(f"Ошибка: {e}", file=sys.stderr)
        logger.log_info(f"ERROR: {e}")
        logger.close()
        sys.exit(1)

    try:
        time.sleep(0.5)

        if args.no_test:
            import threading

            def read_serial():
                buf = b""
                while ser.is_open:
                    chunk = ser.read(ser.in_waiting or 1)
                    if chunk:
                        buf += chunk
                        while b"\r" in buf or b"\n" in buf:
                            sep = b"\r" if b"\r" in buf else b"\n"
                            part, _, buf = buf.partition(sep)
                            try:
                                text = part.decode("utf-8", errors="replace").strip()
                            except Exception:
                                text = ""
                            if text:
                                logger.log_rx(text)
                                print(text)

            logger.log_info("Режим лога (--no-test). Вводите команды, Ctrl+C — выход.")
            t = threading.Thread(target=read_serial, daemon=True)
            t.start()
            while True:
                line = sys.stdin.readline()
                if not line:
                    break
                if line.strip():
                    send_cmd(ser, line.rstrip(), logger)
        else:
            results = run_tests(ser, logger)
            passed = sum(1 for v in results.values() if v)
            total = len(results)
            logger.log_info(f"\nИтого: {passed}/{total} тестов пройдено")
            print(f"\n--- Результат: {passed}/{total} тестов ---")
            for name, ok in results.items():
                print(f"  {'[OK]' if ok else '[FAIL]'} {name}")

    except KeyboardInterrupt:
        logger.log_info("Прервано пользователем")
    finally:
        ser.close()
        logger.close()

    print(f"\nЛог сохранён: {log_path}")


if __name__ == "__main__":
    main()
