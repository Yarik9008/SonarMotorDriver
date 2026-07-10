"""Протокол FW_SonarMotorDriver — единый источник правды.

Здесь собрано всё, что должно совпадать у GUI и у встроенного симулятора:
- константы и диапазоны (из board.h прошивки);
- построители команд (что отправляет GUI);
- клиентские валидаторы (чтобы не попасть в «молчание» прошивки на плохой аргумент);
- классификация и разбор входящих строк (ответ / телеметрия / mcfg);
- симметричная пара format_telemetry / parse_telemetry (формат строго как в прошивке).

Формат строк прошивки (src/main.c):
- debug=0: "cp:%.2f,ec:%u"
- debug=1: "cp:%.2f,tp:%.2f,pe:%.2f,u:%.4f,m:%s,ec:%u,kp:%.4f,ki:%.4f,kd:%.4f,v:%.1f,a:%.1f,of:%lu,drp:%lu"
- mcfg:    "mode=%s run=%u hold=%u microsteps=%u ready=%d"
"""
from __future__ import annotations

from dataclasses import dataclass

# ── Связь ──────────────────────────────────────────────────────────────────
BAUD = 115200

# ── Диапазоны и наборы (board.h / cmd_parser.c) ────────────────────────────
CURRENT_MIN = 0
CURRENT_MAX = 3000          # мА
OP_MIN = 0
OP_MAX = 65535              # период телеметрии, ~мс (0 = выкл)
MSTEP_VALUES = (1, 2, 4, 8, 16, 32, 64, 128, 256)

# ── Механика / энкодер ─────────────────────────────────────────────────────
MAX_SPEED_DEG_S = 1200.0    # аппаратный потолок скорости (board.h)
SPEED_MIN_DEG_S = 1.0       # нижняя граница v= (board.h SPEED_MIN_DEG_S)
ACCEL_MAX_DEG_S2 = 100000.0  # верхняя граница a= (board.h ACCEL_MAX_DEG_S2)

# Пресеты кнопок джога «быстро» / «медленно» (клиентские, GUI)
JOG_FAST_DEG_S = 1200.0
JOG_SLOW_DEG_S = 30.0
MOTOR_FULL_STEPS_REV = 200
TMC2209_MICROSTEPS_DEF = 256
MOTOR_STEPS_PER_REV = MOTOR_FULL_STEPS_REV * TMC2209_MICROSTEPS_DEF   # 51200
ENCODER_COUNTS_REV = 131072  # 2^17
DEG_PER_STEP = 360.0 / MOTOR_STEPS_PER_REV
COUNTS_PER_DEG = ENCODER_COUNTS_REV / 360.0

# ── Легенда кода ошибки ec (board.h ErrCode / biss_c.h) ─────────────────────
EC_LEGEND = {
    0: "OK",
    1: "BiSS CRC",
    2: "нет ответа",
    3: "ошибка датчика",
    4: "предупреждение",
    5: "SPI/HAL",
    6: "выброс (фильтр)",
    7: "блокировка вала",
}


@dataclass
class Defaults:
    """Значения по умолчанию прошивки (board.h) — старт симулятора и «сброс» UI."""
    kp: float = 0.025
    ki: float = 0.0
    kd: float = 0.0
    op_ms: int = 4
    debug: int = 0
    irun: int = 600
    ihold: int = 300
    microsteps: int = 256
    target_deg: float = 0.0
    vmax: float = 1200.0     # предел скорости, °/с (SPEED_DEFAULT_DEG_S)
    accel: float = 2000.0    # предел ускорения, °/с² (0 = выкл; ACCEL_DEFAULT_DEG_S2)


DEFAULTS = Defaults()


# ── Предикаты диапазонов (единый источник для GUI-валидации и симулятора) ───
# Держим границы в одном месте: и клиентская validate(), и FirmwareSimulator
# сверяют аргументы этими функциями — правки диапазона не расходятся по коду.
def speed_ok(v: float) -> bool:
    return SPEED_MIN_DEG_S <= v <= MAX_SPEED_DEG_S


def accel_ok(v: float) -> bool:
    return 0.0 <= v <= ACCEL_MAX_DEG_S2


def op_ok(n: int) -> bool:
    return OP_MIN <= n <= OP_MAX


def current_ok(n: int) -> bool:
    return CURRENT_MIN <= n <= CURRENT_MAX


def mstep_ok(n) -> bool:
    return n in MSTEP_VALUES


# ── Форматирование чисел для команд ────────────────────────────────────────
def fmt_num(v: float) -> str:
    """Компактное число без лишних нулей: 90.0 -> '90', 45.5 -> '45.5'."""
    s = f"{float(v):.4f}".rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


# ── Построители команд (то, что отправляет GUI) ────────────────────────────
def cmd_enable() -> str:                 return "en"
def cmd_disable() -> str:                return "dis"
def cmd_stop() -> str:                   return "stop"
def cmd_target(deg: float) -> str:       return f"t={fmt_num(deg)}"
def cmd_jog(sign: str) -> str:           return f"t={'+' if sign in ('+', 1, '1') else '-'}"
def cmd_kp(v: float) -> str:             return f"kp={fmt_num(v)}"
def cmd_ki(v: float) -> str:             return f"ki={fmt_num(v)}"
def cmd_kd(v: float) -> str:             return f"kd={fmt_num(v)}"
def cmd_speed(v: float) -> str:          return f"v={fmt_num(v)}"
def cmd_accel(v: float) -> str:          return f"a={fmt_num(v)}"
def cmd_op(n: int) -> str:               return f"op={int(n)}"
def cmd_debug(on: bool) -> str:          return f"debug={1 if on else 0}"
def cmd_irun(ma: int) -> str:            return f"irun {int(ma)}"
def cmd_ihold(ma: int) -> str:           return f"ihold {int(ma)}"
def cmd_icur(run: int, hold: int) -> str: return f"icur {int(run)} {int(hold)}"
def cmd_mstep(n: int) -> str:            return f"mstep {int(n)}"
def cmd_mcfg() -> str:                   return "mcfg"
def cmd_diag() -> str:                   return "diag"


def cmd_scan_sector(start: float, end: float, step: float, delay_ms: int) -> str:
    return f"scan={fmt_num(start)},{fmt_num(end)},{fmt_num(step)},{int(delay_ms)}"


def cmd_scan_infinite(start: float, sign: str, step: float, delay_ms: int) -> str:
    s = "+" if sign in ("+", 1, "1") else "-"
    return f"scan={fmt_num(start)},{s},{fmt_num(step)},{int(delay_ms)}"


# ── Клиентская валидация (защита от «молчания» на плохой аргумент) ──────────
def _as_float(s: str):
    try:
        return float(s)
    except ValueError:
        return None


def _as_int(s: str):
    try:
        return int(s)
    except ValueError:
        return None


def validate(cmd: str) -> tuple[bool, str]:
    """Проверяет команду перед отправкой. Возвращает (ok, причина_ошибки).

    Ловит именно те случаи, когда прошивка молча проглотит команду
    (число вне диапазона / не число), чтобы GUI показал причину, а не завис.
    """
    c = cmd.strip()
    if not c:
        return False, "пустая команда"

    if c in ("en", "dis", "stop", "mcfg", "diag", "t=+", "t=-"):
        return True, ""

    if c.startswith("t="):
        return (True, "") if _as_float(c[2:]) is not None else (False, "t=: нужно число градусов")
    for pfx in ("kp=", "ki=", "kd="):
        if c.startswith(pfx):
            return (True, "") if _as_float(c[3:]) is not None else (False, f"{pfx} нужно число")
    if c.startswith("v="):
        n = _as_float(c[2:])
        if n is None or not speed_ok(n):
            return False, f"v: скорость {SPEED_MIN_DEG_S:g}..{MAX_SPEED_DEG_S:g} °/с"
        return True, ""
    if c.startswith("a="):
        n = _as_float(c[2:])
        if n is None or not accel_ok(n):
            return False, f"a: ускорение 0..{ACCEL_MAX_DEG_S2:g} °/с²"
        return True, ""
    if c.startswith("op="):
        n = _as_int(c[3:])
        if n is None or not op_ok(n):
            return False, f"op: целое {OP_MIN}..{OP_MAX}"
        return True, ""
    if c.startswith("debug="):
        return (True, "") if c[6:] in ("0", "1") else (False, "debug: только 0 или 1")
    if c.startswith("scan="):
        return _validate_scan(c[5:])
    if c.startswith("irun "):
        return _validate_current(c[5:], "irun")
    if c.startswith("ihold "):
        return _validate_current(c[6:], "ihold")
    if c.startswith("icur "):
        parts = c[5:].split()
        if len(parts) != 2:
            return False, "icur: нужно два числа: run hold"
        for p in parts:
            n = _as_int(p)
            if n is None or not current_ok(n):
                return False, f"icur: ток {CURRENT_MIN}..{CURRENT_MAX} мА"
        return True, ""
    if c.startswith("mstep "):
        n = _as_int(c[6:])
        if not mstep_ok(n):
            return False, "mstep: 1/2/4/8/16/32/64/128/256"
        return True, ""

    # Неизвестная команда — прошивка честно ответит err:unknown, отправку разрешаем.
    return True, ""


def _validate_current(arg: str, name: str) -> tuple[bool, str]:
    n = _as_int(arg.strip())
    if n is None or not current_ok(n):
        return False, f"{name}: ток {CURRENT_MIN}..{CURRENT_MAX} мА"
    return True, ""


def _validate_scan(arg: str) -> tuple[bool, str]:
    parts = arg.split(",")
    if len(parts) != 4:
        return False, "scan: start,end,step,delay"
    start = _as_float(parts[0])
    step = _as_float(parts[2])
    delay = _as_int(parts[3])
    if start is None or step is None or delay is None:
        return False, "scan: неверные числа"
    if step <= 0 or delay <= 0:
        return False, "scan: step>0 и delay>0"
    if parts[1] in ("+", "-"):
        return True, ""
    end = _as_float(parts[1])
    if end is None:
        return False, "scan: неверный end"
    if not (start < end):
        return False, "scan: нужно start<end (или '+/-')"
    return True, ""


def expects_reply(cmd: str) -> bool:
    """Все непустые команды ждут строку ответа (ok:/err:/mcfg)."""
    return bool(cmd.strip())


# ── Классификация и разбор входящих строк ──────────────────────────────────
def classify_line(line: str) -> str:
    """'telemetry' | 'mcfg' | 'reply' | 'other'."""
    s = line.strip()
    if s.startswith("cp:"):
        return "telemetry"
    if s.startswith("mode="):
        return "mcfg"
    if s.startswith("ok:") or s.startswith("err:"):
        return "reply"
    return "other"


def parse_telemetry(line: str) -> dict | None:
    """Разбирает строку телеметрии (debug=0 или debug=1) в словарь.

    Ключи: cp, tp, pe, u, kp, ki, kd, v, a (float), m (str 'cl'/'ol'),
    ec, of, drp (int). Присутствуют не все поля (зависит от debug).
    None — если это не телеметрия.
    """
    s = line.strip()
    if not s.startswith("cp:"):
        return None
    out: dict = {}
    for tok in s.split(","):
        if ":" not in tok:
            continue
        key, _, val = tok.partition(":")
        key = key.strip()
        val = val.strip()
        if key == "m":
            out["m"] = val
        elif key in ("ec", "of", "drp"):
            iv = _as_int(val)
            if iv is not None:
                out[key] = iv
        else:
            fv = _as_float(val)
            if fv is not None:
                out[key] = fv
    return out if "cp" in out else None


def parse_ok_reply(line: str) -> tuple[str, float | int] | None:
    """Разбирает ok:ответ и возвращает (ключ_состояния, значение) или None."""
    s = line.strip()
    if not s.startswith("ok:"):
        return None
    body = s[3:]
    for pfx, key in (("op=", "op_ms"), ("v=", "v"), ("a=", "a"),
                     ("kp=", "kp"), ("ki=", "ki"), ("kd=", "kd")):
        if body.startswith(pfx):
            raw = body[len(pfx):]
            if key == "op_ms":
                iv = _as_int(raw)
                return (key, iv) if iv is not None else None
            fv = _as_float(raw)
            return (key, fv) if fv is not None else None
    return None


def parse_mcfg(line: str) -> dict | None:
    """Разбирает 'mode=STEP_DIR run=600 hold=300 microsteps=256 ready=1'."""
    s = line.strip()
    if not s.startswith("mode="):
        return None
    out: dict = {}
    for tok in s.split():
        if "=" not in tok:
            continue
        key, _, val = tok.partition("=")
        if key == "mode":
            out["mode"] = val
        else:
            iv = _as_int(val)
            out[key] = iv if iv is not None else val
    return out or None


def format_telemetry(cp: float, tp: float, pe: float, u: float, mode: str,
                     ec: int, kp: float, ki: float, kd: float, drp: int,
                     debug: bool, vmax: float = 1200.0, accel: float = 0.0,
                     outliers: int = 0) -> str:
    """Формирует строку телеметрии строго в формате прошивки."""
    if debug:
        return (f"cp:{cp:.2f},tp:{tp:.2f},pe:{pe:.2f},u:{u:.4f},m:{mode},"
                f"ec:{ec},kp:{kp:.4f},ki:{ki:.4f},kd:{kd:.4f},"
                f"v:{vmax:.1f},a:{accel:.1f},of:{outliers},drp:{drp}")
    return f"cp:{cp:.2f},ec:{ec}"
