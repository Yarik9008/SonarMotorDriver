#!/usr/bin/env python3
"""protocol_test.py — конформанс-тест протокола FW_SonarMotorDriver по UART.

Прогоняет КАЖДУЮ команду протокола и сверяет ТОЧНЫЙ ответ платы:
- валидные команды      → точная строка ok:... / mode=... / enc:ok...;
- аргумент вне диапазона → err:bad arg / err:scan;
- «занят» (скан/вращение) → err:busy stop motor first для diag/mstep;
- неразбираемый аргумент → «молчание» прошивки (парсер вернул 0, ответа нет);
- телеметрия             → формат debug=0/1 разбирается, поля и ec корректны;
- замкнутый контур       → позиция реально сходится к цели.

Единый источник правды по формату — sonar_gui/protocol.py (как у GUI и симулятора).

Плата сейчас несёт имитатор (FW_SonarMotorDriver_Sim): ответы детерминированы.
На «боевой» прошивке отличаются только аппаратно-зависимые ответы (mcfg ready,
diag enc:ok/err:enc, установка токов при неготовом TMC) — соответствующие
проверки сделаны терпимыми.

Запуск:
    python protocol_test.py                # автопоиск порта (COM25 = WCH-Link VCP)
    python protocol_test.py COM25
    python protocol_test.py --list         # показать порты и выйти
    python protocol_test.py --quick        # без длинных пауз скана/движения

ВАЖНО: порт эксклюзивный — закройте Serial Monitor / GUI / PuTTY на этом COM.
"""
from __future__ import annotations

import argparse
import importlib.util
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Нужен pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(2)

# ── protocol.py как единый источник правды (грузим по пути, без импорта пакета) ─
_PROTO = Path(__file__).resolve().parent / "sonar_gui" / "protocol.py"
_spec = importlib.util.spec_from_file_location("sonar_protocol", _PROTO)
P = importlib.util.module_from_spec(_spec)
sys.modules["sonar_protocol"] = P   # dataclass в protocol.py резолвит __module__ через sys.modules
_spec.loader.exec_module(P)

BAUD = P.BAUD

# ── Ожидаемые строки ошибок (src/main.c ProcessCommand) ────────────────────────
ERR_V = f"err:bad arg (v={P.SPEED_MIN_DEG_S:g}..{P.MAX_SPEED_DEG_S:g})"   # err:bad arg (v=1..1200)
ERR_A = f"err:bad arg (a=0..{P.ACCEL_MAX_DEG_S2:g})"                       # err:bad arg (a=0..100000)
ERR_MSTEP = "err:bad arg (1/2/4/8/16/32/64/128/256)"
ERR_SCAN = "err:scan"
ERR_BUSY = "err:busy stop motor first"
ERR_UNKNOWN = "err:unknown"

# enc:ok n=16/16 spread=0.000 pos=<float> — pos непредсказуем, матчим шаблоном
RE_ENC_OK = re.compile(r"enc:ok n=16/16 spread=0\.000 pos=-?\d+\.\d{2}")
SILENCE = None  # маркер «ответа быть не должно»


def _is_reply(s: str) -> bool:
    """Строка — настоящий ответ на команду (ok/err/mcfg/enc), а не телеметрия.

    Всё прочее (cp:… и обрезки телеметрии вроде 'p:0.00,ec:0' или ':0', которые
    остаются после reset_input_buffer посреди строки) в ответ НЕ попадает.
    """
    return s.startswith(("ok:", "err:", "mode=", "enc:"))


# ── Канал: рамочное чтение строк, разделение ответ/телеметрия ──────────────────
class Link:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, BAUD, timeout=0.05)
        self._buf = b""

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def _drain_frames(self) -> list[str]:
        """Достаёт готовые строки из накопленного буфера (разделители \\r / \\n)."""
        out: list[str] = []
        while True:
            i_cr, i_lf = self._buf.find(b"\r"), self._buf.find(b"\n")
            if i_cr == -1 and i_lf == -1:
                return out
            sep = b"\r" if (i_cr >= 0 and (i_lf < 0 or i_cr <= i_lf)) else b"\n"
            part, _, self._buf = self._buf.partition(sep)
            if part:
                s = part.decode("utf-8", errors="replace").strip()
                if s:
                    out.append(s)

    def _pump(self) -> list[str]:
        n = self.ser.in_waiting
        if n:
            self._buf += self.ser.read(n)
        return self._drain_frames()

    def exchange(self, cmd: str, settle: float = 0.30, idle: float = 0.06):
        """Отправляет cmd и слушает settle с. Возвращает (reply, telem).

        reply — строки ok:/err:/mode=/enc: (ответ на команду),
        telem — строки cp:... (телеметрия), отфильтрованы отдельно.
        Ранний выход: как только пришёл ответ и линия молчит idle с.
        """
        self.ser.reset_input_buffer()
        self._buf = b""
        self.ser.write((cmd + "\r\n").encode("utf-8"))

        reply: list[str] = []
        telem: list[str] = []
        t_end = time.monotonic() + settle
        last = time.monotonic()
        while time.monotonic() < t_end:
            got = self._pump()
            if got:
                last = time.monotonic()
                for s in got:
                    (reply if _is_reply(s) else telem).append(s)
            elif reply and (time.monotonic() - last) >= idle and not self.ser.in_waiting:
                break
            else:
                time.sleep(0.004)
        return reply, telem

    def observe(self, secs: float):
        """Просто слушает secs с (для проверок потока телеметрии)."""
        reply: list[str] = []
        telem: list[str] = []
        t_end = time.monotonic() + secs
        while time.monotonic() < t_end:
            for s in self._pump():
                (reply if _is_reply(s) else telem).append(s)
            time.sleep(0.004)
        return reply, telem


# ── Проверки ───────────────────────────────────────────────────────────────────
def _fail(reason: str):
    return False, reason


def evaluate(expect, reply: list[str]) -> tuple[bool, str]:
    """expect: None(=молчание) | str | re.Pattern | [str|Pattern,...] | callable."""
    if expect is SILENCE:
        return (True, "") if not reply else _fail(f"ждали молчание, пришло {reply}")
    if callable(expect) and not isinstance(expect, re.Pattern):
        return expect(reply)
    want = expect if isinstance(expect, list) else [expect]
    if len(reply) != len(want):
        return _fail(f"ждали {len(want)} строк(и) {want}, пришло {reply}")
    for w, got in zip(want, reply):
        if isinstance(w, re.Pattern):
            if not w.fullmatch(got):
                return _fail(f"{got!r} не по шаблону /{w.pattern}/")
        elif w != got:
            return _fail(f"ждали {w!r}, пришло {got!r}")
    return True, ""


def check_mcfg(run: int, hold: int, ms: int) -> Callable:
    def _c(reply: list[str]):
        if len(reply) != 1:
            return _fail(f"mcfg: ждали 1 строку, пришло {reply}")
        m = P.parse_mcfg(reply[0])
        if not m:
            return _fail(f"mcfg не разобран: {reply[0]!r}")
        if m.get("mode") != "STEP_DIR":
            return _fail(f"mode={m.get('mode')} (ждали STEP_DIR)")
        for k, v in (("run", run), ("hold", hold), ("microsteps", ms)):
            if m.get(k) != v:
                return _fail(f"{k}={m.get(k)} (ждали {v})")
        if m.get("ready") not in (0, 1):
            return _fail(f"ready={m.get('ready')} (ждали 0/1)")
        return True, ""
    return _c


def check_telem_brief(reply: list[str]):
    # reply здесь — это telem-строки (см. вызов obs_test)
    if len(reply) < 3:
        return _fail(f"мало телеметрии: {len(reply)} строк")
    for ln in reply[:6]:
        d = P.parse_telemetry(ln)
        if not d:
            return _fail(f"не разобрано: {ln!r}")
        if set(d) != {"cp", "ec"}:
            return _fail(f"поля {sorted(d)} (ждали cp,ec): {ln!r}")
        if d["ec"] != 0:
            return _fail(f"ec={d['ec']} (ждали 0): {ln!r}")
    return True, ""


def check_telem_debug(reply: list[str]):
    need = {"cp", "tp", "pe", "u", "m", "ec", "kp", "ki", "kd", "v", "a", "of", "drp"}
    if len(reply) < 3:
        return _fail(f"мало телеметрии: {len(reply)} строк")
    for ln in reply[:6]:
        d = P.parse_telemetry(ln)
        if not d:
            return _fail(f"не разобрано: {ln!r}")
        miss = need - set(d)
        if miss:
            return _fail(f"нет полей {sorted(miss)}: {ln!r}")
        if d["m"] != "cl":
            return _fail(f"m={d['m']!r} (ждали 'cl')")
        if d["ec"] != 0:
            return _fail(f"ec={d['ec']} (ждали 0)")
    return True, ""


# ── Раннер ─────────────────────────────────────────────────────────────────────
@dataclass
class Runner:
    link: Link
    quick: bool = False
    passed: int = 0
    failed: int = 0
    fails: list[tuple[str, str]] = field(default_factory=list)

    def phase(self, title: str):
        print(f"\n── {title} " + "─" * max(2, 46 - len(title)))

    def t(self, cmd: str, expect, name: str = "", settle: float = 0.30):
        reply, _ = self.link.exchange(cmd, settle=settle)
        ok, reason = evaluate(expect, reply)
        self._record(name or cmd, cmd, ok, reason)

    def obs_test(self, name: str, checker: Callable, secs: float):
        """Проверка потока телеметрии: слушаем secs, отдаём telem-строки в checker."""
        _, telem = self.link.observe(secs)
        ok, reason = checker(telem)
        self._record(name, f"<{secs:.1f}s stream>", ok, reason)

    def expect_no_telemetry(self, name: str, secs: float = 0.5):
        self.link.exchange("", settle=0.05)  # сбросить вход
        _, telem = self.link.observe(secs)
        ok = len(telem) <= 1  # допускаем одну строку «в полёте» на момент op=0
        self._record(name, "<silence>", ok,
                     "" if ok else f"телеметрия не остановилась: {len(telem)} строк")

    def converge(self, target: float, tol: float = 0.6, timeout: float = 3.0):
        """Ждёт, пока cp телеметрии подойдёт к target (нужна включённая телеметрия)."""
        name = f"движение к {target:g}° (cp→{target:g}±{tol:g})"
        t_end = time.monotonic() + (timeout if not self.quick else max(timeout, 1.5))
        last = None
        while time.monotonic() < t_end:
            _, telem = self.link.observe(0.1)
            for ln in telem:
                d = P.parse_telemetry(ln)
                if d and "cp" in d:
                    last = d["cp"]
                    if abs(last - target) <= tol:
                        self._record(name, f"t={target:g}", True, "")
                        return
        self._record(name, f"t={target:g}", False,
                     f"не сошлось: последнее cp={last}")

    def _record(self, name: str, cmd: str, ok: bool, reason: str):
        if ok:
            self.passed += 1
            print(f"  [OK]   {name}")
        else:
            self.failed += 1
            self.fails.append((name, reason))
            print(f"  [FAIL] {name}  ({cmd})\n         {reason}")

    def pause(self, secs: float):
        time.sleep(secs if not self.quick else min(secs, 0.2))


def run_all(link: Link, quick: bool) -> Runner:
    r = Runner(link, quick=quick)

    # Слить стартовый мусор и заглушить телеметрию — чистые ответы на команды.
    link.observe(0.3)
    r.phase("Телеметрия off (op=0)")
    r.t("op=0", "ok:op=0")

    r.phase("Базовые команды")
    r.t("en", "ok:en")
    r.t("dis", "ok:dis")
    r.t("stop", "ok:stop")

    r.phase("Цель t= / непрерывное вращение")
    r.t("t=90", "ok:t=90.00")
    r.t("t=-45.5", "ok:t=-45.50")
    r.t("t=0", "ok:t=0.00")
    r.t("t=+", "ok:t=+")
    r.t("stop", "ok:stop")
    r.t("t=-", "ok:t=-")
    r.t("stop", "ok:stop")

    r.phase("ПИД kp/ki/kd")
    r.t("kp=0.5", "ok:kp=0.5000")
    r.t("ki=0.01", "ok:ki=0.0100")
    r.t("kd=0.001", "ok:kd=0.0010")

    r.phase("Профиль скорости/ускорения v= / a=")
    r.t("v=100", "ok:v=100.0")
    r.t("v=1", "ok:v=1.0")
    r.t("v=1200", "ok:v=1200.0")
    r.t("v=0", ERR_V)
    r.t("v=1201", ERR_V)
    r.t("a=5000", "ok:a=5000.0")
    r.t("a=0", "ok:a=0.0")
    r.t("a=100000", "ok:a=100000.0")
    r.t("a=-1", ERR_A)
    r.t("a=100001", ERR_A)

    r.phase("Период телеметрии op= / debug=")
    r.t("op=10", "ok:op=10")
    r.t("op=65535", "ok:op=65535")
    r.t("op=0", "ok:op=0")
    r.t("op=70000", SILENCE, name="op=70000 → молчание (>65535)")
    r.t("op=-5", SILENCE, name="op=-5 → молчание (<0)")
    r.t("debug=1", "ok:debug=1")
    r.t("debug=0", "ok:debug=0")
    r.t("debug=2", SILENCE, name="debug=2 → молчание")

    r.phase("Неразбираемый аргумент → молчание прошивки")
    r.t("t=abc", SILENCE, name="t=abc → молчание")
    r.t("t=inf", SILENCE, name="t=inf → молчание (не конечное)")
    r.t("kp=xyz", SILENCE, name="kp=xyz → молчание")
    r.t("v=fast", SILENCE, name="v=fast → молчание")
    r.t("op=abc", SILENCE, name="op=abc → молчание")

    r.phase("Токи и микрошаг (мотор стоит)")
    r.t("stop", "ok:stop")
    r.t("irun 800", "ok:irun=800")
    r.t("irun 0", "ok:irun=0")
    r.t("irun 3000", "ok:irun=3000")
    r.t("irun 3001", SILENCE, name="irun 3001 → молчание (>3000)")
    r.t("ihold 350", "ok:ihold=350")
    r.t("icur 700 350", "ok:icur=700,350")
    r.t("icur 700", SILENCE, name="icur 700 → молчание (нет 2-го)")
    r.t("mstep 16", "ok:mstep=16")
    r.t("mstep 256", "ok:mstep=256")
    r.t("mstep 7", ERR_MSTEP)
    r.t("mstep 0", ERR_MSTEP)
    r.t("mstep -1", SILENCE, name="mstep -1 → молчание")

    r.phase("mcfg отражает состояние")
    r.t("irun 750", "ok:irun=750")
    r.t("ihold 250", "ok:ihold=250")
    r.t("mstep 32", "ok:mstep=32")
    r.t("mcfg", check_mcfg(750, 250, 32), name="mcfg = run750 hold250 ms32 ready1")

    r.phase("Скан: сектор / бесконечный / ошибки")
    r.t("scan=0,90,10,100", "ok:scan=0.00,90.00,10.00,100")
    r.t("stop", "ok:stop")
    r.t("scan=0,+,5,50", "ok:scan=0.00,+,5.00,50")
    r.t("stop", "ok:stop")
    r.t("scan=0,-,5,50", "ok:scan=0.00,-,5.00,50")
    r.t("stop", "ok:stop")
    r.t("scan=90,10,5,100", ERR_SCAN, name="scan start>=end → err:scan")
    r.t("scan=0,90,0,100", ERR_SCAN, name="scan step=0 → err:scan")
    r.t("scan=0,90,5,0", ERR_SCAN, name="scan delay=0 → err:scan")
    r.t("scan=0,90,5", SILENCE, name="scan 3 поля → молчание")
    r.t("scan=abc,90,5,100", SILENCE, name="scan нечисло → молчание")

    r.phase("Занятость: diag/mstep при движении → err:busy")
    r.t("en", "ok:en")
    r.t("t=+", "ok:t=+")
    r.t("diag", ERR_BUSY, name="diag во вращении → err:busy")
    r.t("mstep 16", ERR_BUSY, name="mstep во вращении → err:busy")
    r.t("stop", "ok:stop")
    r.t("scan=0,180,10,300", "ok:scan=0.00,180.00,10.00,300")
    r.t("diag", ERR_BUSY, name="diag в скане → err:busy")
    r.t("stop", "ok:stop")

    r.phase("diag на неподвижном валу")
    r.t("stop", "ok:stop")
    r.pause(0.05)
    r.t("diag", ["ok:diag", RE_ENC_OK], name="diag idle → ok:diag + enc:ok", settle=0.45)

    r.phase("Неизвестные команды → err:unknown")
    r.t("xyz", ERR_UNKNOWN)
    r.t("help", ERR_UNKNOWN)
    r.t("foo123", ERR_UNKNOWN)

    r.phase("Поток телеметрии — формат")
    r.t("debug=0", "ok:debug=0")
    r.t("op=20", "ok:op=20")
    r.obs_test("телеметрия debug=0: cp,ec (ec=0)", check_telem_brief, 0.7)
    r.t("debug=1", "ok:debug=1")
    r.obs_test("телеметрия debug=1: все поля, m=cl", check_telem_debug, 0.7)
    r.t("op=0", "ok:op=0")
    r.t("debug=0", "ok:debug=0")
    r.expect_no_telemetry("op=0 останавливает поток", secs=0.5)

    r.phase("Замкнутый контур — реальное движение")
    r.t("en", "ok:en")
    r.t("a=0", "ok:a=0.0")
    r.t("v=1200", "ok:v=1200.0")
    r.t("op=20", "ok:op=20")
    r.t("t=0", "ok:t=0.00")
    r.converge(0.0)
    r.t("t=90", "ok:t=90.00")
    r.converge(90.0)
    r.t("t=-90", "ok:t=-90.00")
    r.converge(-90.0)
    r.t("op=0", "ok:op=0")

    r.phase("Восстановление значений по умолчанию")
    r.t("stop", "ok:stop")
    r.t("dis", "ok:dis")
    d = P.DEFAULTS
    r.t(f"kp={P.fmt_num(d.kp)}", f"ok:kp={d.kp:.4f}")
    r.t(f"ki={P.fmt_num(d.ki)}", f"ok:ki={d.ki:.4f}")
    r.t(f"kd={P.fmt_num(d.kd)}", f"ok:kd={d.kd:.4f}")
    r.t(f"v={P.fmt_num(d.vmax)}", f"ok:v={d.vmax:.1f}")
    r.t(f"a={P.fmt_num(d.accel)}", f"ok:a={d.accel:.1f}")
    r.t(f"irun {d.irun}", f"ok:irun={d.irun}")
    r.t(f"ihold {d.ihold}", f"ok:ihold={d.ihold}")
    r.t(f"mstep {d.microsteps}", f"ok:mstep={d.microsteps}")
    r.t(f"op={d.op_ms}", f"ok:op={d.op_ms}")
    return r


# ── Порт ────────────────────────────────────────────────────────────────────────
def find_port() -> str | None:
    """WCH-Link VCP (1a86) / STM32 (0483) / CH340 / CP210x / FTDI."""
    for p in list_ports.comports():
        hwid = (p.hwid or "").lower()
        desc = (p.description or "").lower()
        if any(v in hwid for v in ("1a86", "0483", "ch340", "cp210", "0403")) or "stm" in desc:
            return p.device
    return None


def print_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("COM-портов не найдено.")
        return
    for p in ports:
        print(f"  {p.device:8}  {p.description or '?'}  [{p.hwid or '?'}]")


def main() -> int:
    ap = argparse.ArgumentParser(description="Конформанс-тест протокола FW_SonarMotorDriver")
    ap.add_argument("port", nargs="?", help="COM-порт (по умолчанию — автопоиск)")
    ap.add_argument("--list", action="store_true", help="показать порты и выйти")
    ap.add_argument("--quick", action="store_true", help="без длинных пауз скана/движения")
    args = ap.parse_args()

    if args.list:
        print_ports()
        return 0

    port = args.port or find_port()
    if not port:
        print("Порт не найден. Доступные:", file=sys.stderr)
        print_ports()
        print("\nУкажите явно: python protocol_test.py COM25", file=sys.stderr)
        return 2

    print(f"Порт: {port} @ {BAUD}   (закройте другие мониторы этого COM!)")
    try:
        link = Link(port)
    except serial.SerialException as e:
        msg = str(e)
        print(f"Ошибка открытия {port}: {msg}", file=sys.stderr)
        if "access is denied" in msg.lower() or "permissionerror" in msg.lower():
            print("Порт занят — закройте Serial Monitor / GUI / PuTTY.", file=sys.stderr)
        return 2

    try:
        r = run_all(link, quick=args.quick)
    except KeyboardInterrupt:
        print("\nПрервано.", file=sys.stderr)
        return 130
    finally:
        link.close()

    total = r.passed + r.failed
    print("\n" + "═" * 52)
    print(f"ИТОГО: {r.passed}/{total} пройдено, {r.failed} провалено")
    if r.fails:
        print("\nПровалы:")
        for name, reason in r.fails:
            print(f"  - {name}: {reason}")
    print("═" * 52)
    return 0 if r.failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
