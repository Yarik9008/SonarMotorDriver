#!/usr/bin/env python3
"""Монитор COM-порта для Test_TMC2209. Читает телеметрию, проверяет корректность данных TMC2209."""

import sys
import re
import threading

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Нужен pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

BAUD = 115200

# Ожидаемые строки телеметрии для проверки
EXPECTED_REGISTERS = [
    "GSTAT", "IFCNT", "IOIN", "TSTEP", "SG_RESULT", "MSCNT",
    "MSCURACT", "CHOPCONF", "DRV_STATUS", "PWM_SCALE"
]

# TMC2209 chip version (должен быть 0x21)
TMC2209_VERSION = 0x21

# 128 microsteps -> MRES=1
EXPECTED_MRES = 1


def find_stm32_port():
    """Находит COM-порт STM32 (USB CDC)."""
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        # STM32 CDC, STMicroelectronics, или по VID (0483)
        if "0483" in hwid or "stm" in desc or "stm32" in desc or "cdc" in desc:
            return p.device
    return None


def parse_and_validate(line):
    """Парсит строку и проверяет корректность. Возвращает (ok, message)."""
    line = line.strip()
    if not line:
        return True, ""

    # init ok
    if "init ok" in line:
        if "128 mstep" in line and "1A" in line:
            return True, "[OK] init ok (128 mstep, 1A)"
        return False, f"[WARN] init ok в неожиданном формате: {line}"

    # init FAIL
    if "init FAIL" in line:
        return False, f"[FAIL] {line}"

    # GSTAT
    m = re.match(r"GSTAT=0x([0-9a-fA-F]+)\s+reset=(\d+)\s+drv_err=(\d+)\s+uv_cp=(\d+)", line)
    if m:
        return True, f"[OK] GSTAT: {line}"

    # IFCNT (0-255)
    m = re.match(r"IFCNT=(\d+)", line)
    if m:
        v = int(m.group(1))
        ok = 0 <= v <= 255
        return ok, f"[{'OK' if ok else 'WARN'}] IFCNT={v}"

    # IOIN — проверяем VERSION
    m = re.search(r"IOIN=0x[0-9a-fA-F]+\s+.*VERSION=0x([0-9a-fA-F]+)", line)
    if m:
        ver = int(m.group(1), 16)
        ok = ver == TMC2209_VERSION
        return ok, f"[{'OK' if ok else 'FAIL'}] IOIN VERSION=0x{ver:02X} (ожидается 0x{TMC2209_VERSION:02X})"

    if line.startswith("IOIN="):
        return True, f"[OK] IOIN: {line[:60]}..."

    # TSTEP (0..0xFFFFF)
    m = re.match(r"TSTEP=(\d+)", line)
    if m:
        v = int(m.group(1))
        return True, f"[OK] TSTEP={v}"

    # SG_RESULT (0..1023)
    m = re.match(r"SG_RESULT=(\d+)", line)
    if m:
        v = int(m.group(1))
        ok = 0 <= v <= 1023
        return ok, f"[{'OK' if ok else 'WARN'}] SG_RESULT={v}"

    # MSCNT (0..1023)
    m = re.match(r"MSCNT=(\d+)", line)
    if m:
        v = int(m.group(1))
        ok = 0 <= v <= 1023
        return ok, f"[{'OK' if ok else 'WARN'}] MSCNT={v}"

    # MSCURACT
    m = re.match(r"MSCURACT CUR_A=(-?\d+)\s+CUR_B=(-?\d+)", line)
    if m:
        a, b = int(m.group(1)), int(m.group(2))
        ok = -256 <= a <= 255 and -256 <= b <= 255
        return ok, f"[{'OK' if ok else 'WARN'}] MSCURACT CUR_A={a} CUR_B={b}"

    # CHOPCONF — проверяем MRES (128 mstep -> MRES=1)
    m = re.search(r"CHOPCONF=0x[0-9a-fA-F]+\s+MRES=(\d+)", line)
    if m:
        mres = int(m.group(1))
        ok = mres == EXPECTED_MRES
        return ok, f"[{'OK' if ok else 'WARN'}] CHOPCONF MRES={mres} (ожидается {EXPECTED_MRES})"

    if line.startswith("CHOPCONF="):
        return True, f"[OK] CHOPCONF"

    # DRV_STATUS
    if line.startswith("DRV_STATUS="):
        return True, "[OK] DRV_STATUS"

    # PWM_SCALE
    if line.startswith("PWM_SCALE"):
        return True, "[OK] PWM_SCALE"

    # Приветствие
    if "TMC2209 ready" in line:
        return True, "[OK] TMC2209 ready"

    # Команды-ответы
    if line in ("init", "stop", "on", "off", "h") or line.startswith("mode:") or line.startswith("UA=") or line.startswith("SD="):
        return True, ""

    if line.startswith("IC="):
        m = re.search(r"IC=0x([0-9a-fA-F]+)", line)
        if m:
            ver = int(m.group(1), 16)
            ok = ver == TMC2209_VERSION
            return ok, f"[{'OK' if ok else 'WARN'}] IC version=0x{ver:02X}"

    return True, ""  # неизвестная строка — просто показываем


def read_serial(ser, stop_event, results):
    """Поток: читает порт и выводит строки с проверкой."""
    buf = b""
    found_regs = set()
    errors = []

    while not stop_event.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            break
        if chunk:
            buf += chunk
        while b"\r" in buf or b"\n" in buf:
            line, sep, rest = buf.partition(b"\r")
            if not sep:
                line, sep, rest = buf.partition(b"\n")
            buf = rest
            try:
                text = line.decode("utf-8", errors="replace").strip()
            except Exception:
                text = ""
            if not text:
                continue

            ok, msg = parse_and_validate(text)
            if msg:
                print(msg if msg else text)
            else:
                print(text)

            for reg in EXPECTED_REGISTERS:
                if reg + "=" in text or reg + " " in text:
                    found_regs.add(reg)
                    break
            if not ok and "FAIL" in (msg or ""):
                errors.append(text)

    results["found"] = found_regs
    results["errors"] = errors


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    if not port:
        port = find_stm32_port()
        if port:
            print(f"Найден порт STM32: {port}")
        else:
            print("Доступные порты:")
            for p in list_ports.comports():
                print(f"  {p.device} — {p.description} ({p.hwid})")
            port = input("Введите COM-порт (например COM3): ").strip()
            if not port:
                sys.exit(0)

    print(f"Подключение к {port} @ {BAUD}...")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"Ошибка открытия порта: {e}", file=sys.stderr)
        if "Permission" in str(e) or "Access" in str(e):
            print("Порт занят. Закройте PlatformIO Serial Monitor, PuTTY и др.", file=sys.stderr)
        sys.exit(1)

    print("Монитор запущен. Ввод команд: h(help) i(init) m<N>(move) s(stop) e/d c v st p u")
    print("Ctrl+C для выхода.")
    print("-" * 50)

    stop = threading.Event()
    results = {"found": set(), "errors": []}
    reader = threading.Thread(target=read_serial, args=(ser, stop, results))
    reader.daemon = True
    reader.start()

    try:
        while True:
            try:
                line = sys.stdin.readline()
            except KeyboardInterrupt:
                break
            if not line:
                break
            line = line.strip()
            if not line:
                continue
            cmd = (line + "\r\n").encode("utf-8")
            try:
                ser.write(cmd)
            except serial.SerialException:
                break
    except KeyboardInterrupt:
        pass

    stop.set()
    reader.join(timeout=1)

    # Итог проверки
    found = results.get("found", set())
    missing = set(EXPECTED_REGISTERS) - found
    if missing:
        print("-" * 50)
        print(f"Не получены регистры: {', '.join(sorted(missing))}")
    else:
        print("-" * 50)
        print("Все ожидаемые регистры телеметрии получены.")

    if results.get("errors"):
        print("Обнаружены ошибки:")
        for e in results["errors"]:
            print(f"  {e}")

    ser.close()
    print("Готово.")


if __name__ == "__main__":
    main()
