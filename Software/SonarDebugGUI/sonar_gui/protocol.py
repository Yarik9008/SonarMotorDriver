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
- кадр по приходу в цель (om=1/2) — та же строка с ",ev:1" в конце
- mcfg:    "mode=%s run=%u hold=%u microsteps=%u ready=%d"
- sync:    "sync=%u in=%u out=%u n=%lu"
- om:      "om=%u"      — ответ на запрос om (0..2), без префикса ok:
- hold:    "hold=%u"    — ответ на запрос hold (0|1), без префикса ok:
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# ── Связь ──────────────────────────────────────────────────────────────────
BAUD = 115200

# ── Диапазоны и наборы (board.h / cmd_parser.c) ────────────────────────────
CURRENT_MIN = 0
CURRENT_MAX = 3000          # мА
OP_MIN = 0
OP_MAX = 65535              # период телеметрии, ~мс (0 = выкл)
MSTEP_VALUES = (1, 2, 4, 8, 16, 32, 64, 128, 256)
# Верхняя граница аргумента mstep у ПАРСЕРА прошивки: Cmd_Result.microsteps —
# uint16, и cmd_parser.c проверяет предел ДО приведения типа
# (`v < 0 || v > 65535`), поэтому «mstep 65537» парсер отвергает целиком и
# прошивка молчит, а не выполняет усечённое до 1 значение. Набор
# MSTEP_VALUES — уже свойство чипа: его проверяет обработчик команды и на
# неподходящее отвечает err:bad arg. Две границы разные, и путать их нельзя.
MSTEP_ARG_MAX = 65535

# Режимы синхронизации скана (sync=N): источник перехода к следующей точке
SYNC_MODE_VALUES = (0, 1, 2)
SYNC_MODE_LABELS = {
    0: "таймер (пауза)",
    1: "внешний (SYNC_IN)",
    2: "SYNC_IN + тайм-аут",
}

# Режимы выдачи телеметрии (om=N): источник кадра
OUTPUT_MODE_VALUES = (0, 1, 2)
OUTPUT_MODE_LABELS = {
    0: "по периоду op=",
    1: "по приходу в цель",
    2: "оба источника",
}

# Пауза в точке скана хранится в прошивке как uint16 (Cmd_Result.scan_delay_ms),
# поэтому delay > 65535 парсер отвергает так же, как нечитаемое число, — молча.
SCAN_DELAY_MIN = 1
SCAN_DELAY_MAX = 65535

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

# ── Стартовая диагностика энкодера (board.h ENCODER_DIAG_*) ────────────────
# Вердикт «диагностика пройдена» прошивка выносит по двум условиям
# (main.c EncDiag_Finish): валидных чтений не меньше ENCODER_DIAG_MIN_OK из
# ENCODER_DIAG_SAMPLES и разброс позиции не больше ENCODER_DIAG_MAX_SPREAD_DEG.
# Числа держим здесь, потому что по ним конформанс-тест разбирает строку
# enc:ok: требовать от исправной платы ровно 16/16 и spread=0.000 нельзя —
# один отсчёт 17-битного энкодера это 360/131072 ≈ 0.0027°, и живой вал
# показывает spread=0.003 уже от шума младшего бита.
ENCODER_DIAG_SAMPLES = 16
ENCODER_DIAG_MIN_OK = 12
ENCODER_DIAG_MAX_SPREAD_DEG = 0.5

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


# ── Баннер перезапуска платы (main.c Boot_Banner) ──────────────────────────
# Плата шлёт его ровно один раз, первой строкой после подъёма UART:
# «boot:rst=<флаги> fw=<версия>». Флаги — причины сброса из RCC->CSR через
# запятую в фиксированном порядке (por,pin,sft,iwdg,wwdg,lpwr) либо «-», если
# ни один бит взведён не был. Формат тот же у прошивки-имитатора и у модели
# (simulator.py boot_lines), отличается только значение fw=.
BOOT_PREFIX = "boot:"

# Причины сброса словами — для сообщения оператору. Ключи взяты из
# Boot_Banner: имя флага пишется в строку как есть.
RESET_FLAG_LABELS = {
    "por": "подача питания",
    "pin": "вывод NRST",
    "sft": "программный сброс",
    "iwdg": "сторожевой таймер IWDG",
    "wwdg": "оконный сторожевой таймер WWDG",
    "lpwr": "сбой режима пониженного потребления",
    "-": "причина не указана",
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


def output_mode_ok(n) -> bool:
    return n in OUTPUT_MODE_VALUES


def scan_delay_ok(n) -> bool:
    return SCAN_DELAY_MIN <= n <= SCAN_DELAY_MAX


# ── Кольцевая координата (main.c wrap360 / shortest_path_err) ───────────────
# Прошивка работает в пределах одного оборота: позиция и цель всегда в
# [0,360), ход к цели — кратчайшим путём. Держим ту же арифметику здесь,
# чтобы GUI, валидация и симулятор считали углы одинаково.
def wrap360(deg: float) -> float:
    """Приводит угол к [0, 360). Значение вне диапазона не ошибка: t=370 -> 10."""
    d = math.fmod(deg, 360.0)
    if d < 0.0:
        d += 360.0
    if d >= 360.0:      # -1e-15 после сложения округляется ровно до 360.0
        d = 0.0
    return d


def wrap180(deg: float) -> float:
    """Приводит разность углов к (-180, 180] — ошибка по кратчайшему пути."""
    d = math.fmod(deg + 180.0, 360.0)
    if d <= 0.0:
        d += 360.0
    return d - 180.0


def scan_span(start: float, end: float) -> float:
    """Протяжённость сектора от start в сторону возрастания угла по кольцу.

    end < start задаёт сектор через ноль (350,10 -> 20°). Возвращает 0.0 для
    вырожденного сектора (start и end — одна точка кольца, в т.ч. 0 и 360),
    который прошивка отвергает как err:scan.
    """
    return wrap360(end - start)


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
def cmd_output_mode(n: int) -> str:      return f"om={int(n)}"
def cmd_hold(on: bool) -> str:           return f"hold={1 if on else 0}"
def cmd_debug(on: bool) -> str:          return f"debug={1 if on else 0}"
def cmd_irun(ma: int) -> str:            return f"irun {int(ma)}"
def cmd_ihold(ma: int) -> str:           return f"ihold {int(ma)}"
def cmd_icur(run: int, hold: int) -> str: return f"icur {int(run)} {int(hold)}"
def cmd_mstep(n: int) -> str:            return f"mstep {int(n)}"
def cmd_mcfg() -> str:                   return "mcfg"
def cmd_diag() -> str:                   return "diag"
def cmd_sync_mode(n: int) -> str:        return f"sync={int(n)}"
def cmd_sync_query() -> str:             return "sync"
# Запросы состояния без аргумента: отвечают одной строкой данных (om=N /
# hold=N), без префикса ok: — как запрос sync (cmd_parser.c: strcmp с "om" и
# "hold", main.c CMD_GET_OUTPUT_MODE / CMD_GET_HOLD).
def cmd_output_mode_query() -> str:      return "om"
def cmd_hold_query() -> str:             return "hold"


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


def _as_finite_float(s: str):
    """Число, которое примет прошивка: inf/nan она отвергает (isfinite в
    parse_float() cmd_parser.c) и молчит — значит и нам их пропускать нельзя."""
    v = _as_float(s)
    return v if v is not None and math.isfinite(v) else None


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

    # Команды целиком, аргумента у них нет (в т.ч. запросы состояния
    # sync / om / hold — прошивка узнаёт их strcmp-ом, cmd_parser.c:240-260).
    if c in ("en", "dis", "stop", "mcfg", "diag", "sync", "om", "hold",
             "t=+", "t=-"):
        return True, ""

    if c.startswith("t="):
        return (True, "") if _as_finite_float(c[2:]) is not None else (False, "t=: нужно число градусов")
    for pfx in ("kp=", "ki=", "kd="):
        if c.startswith(pfx):
            return (True, "") if _as_finite_float(c[3:]) is not None else (False, f"{pfx} нужно число")
    if c.startswith("v="):
        n = _as_finite_float(c[2:])
        if n is None or not speed_ok(n):
            return False, f"v: скорость {SPEED_MIN_DEG_S:g}..{MAX_SPEED_DEG_S:g} °/с"
        return True, ""
    if c.startswith("a="):
        n = _as_finite_float(c[2:])
        if n is None or not accel_ok(n):
            return False, f"a: ускорение 0..{ACCEL_MAX_DEG_S2:g} °/с²"
        return True, ""
    if c.startswith("op="):
        n = _as_int(c[3:])
        if n is None or not op_ok(n):
            return False, f"op: целое {OP_MIN}..{OP_MAX}"
        return True, ""
    if c.startswith("om="):
        n = _as_int(c[3:])
        if n is None or not output_mode_ok(n):
            return False, "om: только 0, 1 или 2"
        return True, ""
    if c.startswith("hold="):
        return (True, "") if c[5:] in ("0", "1") else (False, "hold: только 0 или 1")
    if c.startswith("debug="):
        return (True, "") if c[6:] in ("0", "1") else (False, "debug: только 0 или 1")
    if c.startswith("sync="):
        return (True, "") if c[5:] in ("0", "1", "2") else (False, "sync: только 0, 1 или 2")
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
    start = _as_finite_float(parts[0])
    step = _as_finite_float(parts[2])
    delay = _as_int(parts[3])
    if start is None or step is None or delay is None:
        return False, "scan: неверные числа"
    if step <= 0:
        return False, "scan: step>0"
    if not scan_delay_ok(delay):
        return False, f"scan: delay {SCAN_DELAY_MIN}..{SCAN_DELAY_MAX} мс"
    if parts[1] in ("+", "-"):
        return True, ""
    end = _as_finite_float(parts[1])
    if end is None:
        return False, "scan: неверный end"
    # end отсчитывается от start по кольцу, поэтому end < start — это сектор
    # через ноль (350,10 -> 20°), а не ошибка. Нельзя только вырожденный
    # сектор, где start и end — одна точка кольца (в т.ч. 0 и 360).
    if scan_span(start, end) <= 0.0:
        return False, "scan: start и end — одна точка (полный круг задаётся '+/-')"
    return True, ""


def expects_reply(cmd: str) -> bool:
    """Все непустые команды ждут строку ответа (ok:/err:/mcfg)."""
    return bool(cmd.strip())


# ── Классификация и разбор входящих строк ──────────────────────────────────
def classify_line(line: str) -> str:
    """'telemetry' | 'mcfg' | 'sync' | 'reply' | 'other'.

    Ответы на запросы состояния (`om`, `hold`) приходят без префикса ok: —
    одной строкой «om=N» / «hold=N» (main.c CMD_GET_OUTPUT_MODE и
    CMD_GET_HOLD: SendResponse("om=%u\\r\\n") / ("hold=%u\\r\\n")). Это
    ОТВЕТ на команду, а не телеметрия: контроллер обязан снять по нему
    ожидание ответа, иначе запрос упрётся в таймаут. Разбирает строку
    parse_state_reply().

    Баннер перезапуска «boot:rst=… fw=…», наоборот, остаётся в 'other':
    плата шлёт его сама, без команды, и очередь ожидания он трогать не
    должен. Узнаётся он parse_boot() / is_boot_line().
    """
    s = line.strip()
    if s.startswith("cp:"):
        return "telemetry"
    if s.startswith("mode="):
        return "mcfg"
    if s.startswith("sync="):
        return "sync"
    if s.startswith("om=") or s.startswith("hold="):
        return "reply"
    if s.startswith("ok:") or s.startswith("err:"):
        return "reply"
    return "other"


def parse_telemetry(line: str) -> dict | None:
    """Разбирает строку телеметрии (debug=0 или debug=1) в словарь.

    Ключи: cp, tp, pe, u, kp, ki, kd, v, a (float), m (str 'cl'/'ol'),
    ec, of, drp, ev (int). Присутствуют не все поля (зависит от debug);
    ev=1 стоит только в кадре по приходу в цель (om=1/2).
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
        elif key in ("ec", "of", "drp", "ev"):
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
    for pfx, key in (("op=", "op_ms"), ("sync=", "sync_mode"), ("v=", "v"),
                     ("a=", "a"), ("kp=", "kp"), ("ki=", "ki"), ("kd=", "kd")):
        if body.startswith(pfx):
            raw = body[len(pfx):]
            if key in ("op_ms", "sync_mode"):
                iv = _as_int(raw)
                return (key, iv) if iv is not None else None
            fv = _as_float(raw)
            return (key, fv) if fv is not None else None
    return None


def parse_state_reply(line: str) -> tuple[str, int] | None:
    """Разбирает ответ на запрос состояния: 'om=N' → ('output_mode', N),
    'hold=N' → ('hold', N). None — если строка не такая.

    Ключи совпадают с полями DeviceState, как у parse_ok_reply, поэтому
    подтверждение из запроса ложится в состояние тем же путём, что и эхо
    ok:om= / ok:hold= на установку. Значение проверяется по допустимому
    набору: строку с чужим числом (om=7) прошивка выдать не может, и молча
    записывать её в состояние нельзя.
    """
    s = line.strip()
    for pfx, key, allowed in (("om=", "output_mode", OUTPUT_MODE_VALUES),
                              ("hold=", "hold", (0, 1))):
        if s.startswith(pfx):
            iv = _as_int(s[len(pfx):])
            return (key, iv) if iv in allowed else None
    return None


def parse_sync(line: str) -> dict | None:
    """Разбирает 'sync=1 in=0 out=1 n=42' (ответ на запрос `sync`).

    Ключи результата — как поля DeviceState: sync_mode, sync_in, sync_out,
    sync_edges (все int). None — если строка не похожа на статус sync.
    """
    s = line.strip()
    if not s.startswith("sync="):
        return None
    keymap = {"sync": "sync_mode", "in": "sync_in",
              "out": "sync_out", "n": "sync_edges"}
    out: dict = {}
    for tok in s.split():
        key, _, val = tok.partition("=")
        if key in keymap:
            iv = _as_int(val)
            if iv is not None:
                out[keymap[key]] = iv
    return out if "sync_mode" in out else None


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


def is_boot_line(line: str) -> bool:
    """Строка — баннер перезапуска платы (`boot:...`)."""
    return line.strip().startswith(BOOT_PREFIX)


def parse_boot(line: str) -> dict | None:
    """Разбирает 'boot:rst=iwdg fw=1.0.0' → {'rst': 'iwdg', 'fw': '1.0.0'}.

    None — если строка не баннер. Поля, которых в строке нет, в словарь не
    попадают: баннер прошивки всегда несёт оба, но разбор не должен падать на
    обрезке строки (reset_input_buffer режет линию посреди кадра).

    Класс строки (classify_line) остаётся 'other' сознательно: баннер плата
    шлёт сама, а не в ответ на команду, и снимать им ожидание ответа из FIFO
    контроллера нельзя — иначе перезагрузка платы «съела» бы ответ на чужую
    команду. Кому баннер нужен, тот спрашивает этой функцией (controller.py).
    """
    s = line.strip()
    if not s.startswith(BOOT_PREFIX):
        return None
    out: dict = {}
    for tok in s[len(BOOT_PREFIX):].split():
        key, sep, val = tok.partition("=")
        if sep == "=" and key in ("rst", "fw"):
            out[key] = val
    return out


def boot_reason_text(rst: str | None) -> str:
    """Флаги сброса словами: 'iwdg' → 'сторожевой таймер IWDG'.

    Несколько флагов идут через запятую в том же порядке, что в строке
    (Boot_Banner складывает их в por,pin,sft,iwdg,wwdg,lpwr). Незнакомый
    флаг показываем как есть — новую причину лучше увидеть, чем потерять.
    """
    if not rst:
        return RESET_FLAG_LABELS["-"]
    parts = [RESET_FLAG_LABELS.get(f, f) for f in rst.split(",") if f]
    return ", ".join(parts) if parts else RESET_FLAG_LABELS["-"]



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
