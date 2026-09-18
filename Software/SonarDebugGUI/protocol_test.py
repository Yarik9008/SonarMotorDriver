#!/usr/bin/env python3
"""protocol_test.py — конформанс-тест протокола FW_SonarMotorDriver по UART.

Прогоняет КАЖДУЮ команду протокола и сверяет ТОЧНЫЙ ответ платы:
- валидные команды      → точная строка ok:... / mode=...;
- аргумент вне диапазона → err:bad arg / err:scan;
- «занят» (скан/вращение) → err:busy stop motor first для diag/mstep;
- неразбираемый аргумент → «молчание» прошивки (парсер вернул 0, ответа нет);
  молчание проверяется строго: любая строка, не являющаяся корректным кадром
  телеметрии, валит проверку — см. _classify();
- телеметрия             → формат debug=0/1 разбирается, поля и ec корректны;
- замкнутый контур       → позиция реально сходится к цели.

Единый источник правды по формату — sonar_gui/protocol.py (как у GUI и симулятора).

Если плата несёт имитатор (сборка `pio run -e sim` той же прошивки), ответы
детерминированы.
На «боевой» прошивке отличаются только аппаратно-зависимые ответы, и терпимы
они ровно настолько, насколько того требует прошивка:
- enc:ok — разбирается по критериям самой диагностики (check_enc_ok): она
  проходит при 12 валидных чтениях из 16 и печатает измеренный разброс, так
  что буквальных «16/16 spread=0.000» от исправной платы ждать нельзя;
- irun/ihold/icur/mstep — при молчащем по UART TMC2209 прошивка отвечает
  err:not ready вместо любой другой строки (tmc_reply); в режиме --sim
  поблажка не действует, там сверяется точный ответ;
- mcfg — поле ready берётся как есть, а при ready=0 не с чем сверять токи
  (их применить не удалось), см. check_mcfg.

Режим --sim гоняет ровно тот же список проверок против Python-модели прошивки
(sonar_gui/simulator.py) на виртуальных часах: без платы, без COM-порта и без
ожиданий в реальном времени. Прогон детерминирован и занимает секунды, поэтому
годится для CI; аппаратно-зависимые ветки (неготовый TMC2209, живой энкодер,
фронты SYNC_IN) в модели недостижимы — что именно она упрощает, перечислено в
шапке simulator.py.

Запуск:
    python protocol_test.py                # автопоиск порта (COM25 = WCH-Link VCP)
    python protocol_test.py COM25
    python protocol_test.py --sim          # против модели прошивки, без железа
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
    # Режиму --sim pyserial не нужен: отсутствие пакета валит только работу
    # с реальным портом (проверка в main()).
    serial = None
    list_ports = None

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

# enc:ok n=<ok>/<всего> spread=<град> pos=<град> — разбирается check_enc_ok()
RE_ENC_OK_FMT = re.compile(
    r"enc:ok n=(\d+)/(\d+) spread=(\d+\.\d{3}) pos=(-?\d+\.\d{2})")
# sync=<0..2> in=<0/1> out=<0/1> n=<счётчик> — счётчик фронтов непредсказуем
RE_SYNC = re.compile(r"sync=[0-2] in=[01] out=[01] n=\d+")
SILENCE = None  # маркер «ответа быть не должно»


# ── Классификация входящих строк ──────────────────────────────────────────────
# По UART приходит три вида строк: ответы на команды, кадры телеметрии и
# обрезки кадра (reset_input_buffer режет линию посреди строки). Делить их по
# списку известных префиксов нельзя: строка с префиксом, которого в списке
# нет (warn:, info:, новая диагностика), утекала бы в телеметрию и молча
# выбрасывалась, а восемнадцать проверок вида r.t("om=3", SILENCE) остались бы
# зелёными. Поэтому телеметрия — это только РАЗОБРАННЫЙ кадр, обрезок — только
# настоящий хвост кадра, а всё остальное считается ответом и валит SILENCE.
#
# Ответы на запросы состояния (om=N, hold=N) префикса ok: не имеют — это
# такие же строки данных, как ответ на sync (main.c CMD_GET_OUTPUT_MODE и
# CMD_GET_HOLD). В «ответы» их уводит и правило по умолчанию, но перечислены
# они явно: правило существует для строк с НЕИЗВЕСТНЫМ префиксом, а эти два
# известны и составляют часть протокола.
REPLY_PREFIXES = ("ok:", "err:", "mode=", "sync=", "om=", "hold=",
                  "enc:", "boot:")
# Поля кадра телеметрии (см. format_telemetry в protocol.py)
_TELEM_KEYS = ("cp", "tp", "pe", "u", "m", "ec", "kp", "ki", "kd",
               "v", "a", "of", "drp", "ev")
# Как ключ выглядит в ОБРЕЗКЕ: целиком, обрубленным с начала ("p" от "cp")
# или отсутствующим вовсе (строка ":0").
_TELEM_KEY_TAILS = {k[i:] for k in _TELEM_KEYS for i in range(len(k))} | {""}


def _is_telemetry(s: str) -> bool:
    d = P.parse_telemetry(s)
    return bool(d) and "cp" in d


def _is_telemetry_scrap(s: str) -> bool:
    """Хвост кадра телеметрии: 'p:0.00,ec:0' или ':0' после сброса буфера.

    Обрезок начинается на обрубленном ключе (или без ключа), а все остальные
    его поля — обычные поля кадра. Строка с чужим ключом ('warn:hot')
    обрезком не считается: она уйдёт в ответы, где её увидит проверка молчания.
    """
    parts = s.split(",")
    key, sep, _ = parts[0].partition(":")
    if sep != ":" or key not in _TELEM_KEY_TAILS:
        return False
    return all(":" in p and p.partition(":")[0] in _TELEM_KEYS for p in parts[1:])


def _classify(s: str) -> str:
    """Куда отнести строку: 'reply' (ответ), 'telem' (кадр) или 'scrap' (обрезок)."""
    if s.startswith(REPLY_PREFIXES):
        return "reply"
    if _is_telemetry(s):
        return "telem"
    return "scrap" if _is_telemetry_scrap(s) else "reply"


def _sort_lines(lines, reply: list[str], telem: list[str]) -> None:
    """Раскладывает пришедшие строки по спискам; обрезки отбрасывает."""
    for s in lines:
        kind = _classify(s)
        if kind == "reply":
            reply.append(s)
        elif kind == "telem":
            telem.append(s)


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

    # Часы и паузы канала: раннер не зовёт time.* напрямую, чтобы тот же код
    # работал и на виртуальных часах SimLink.
    def now(self) -> float:
        return time.monotonic()

    def pause(self, secs: float) -> None:
        time.sleep(secs)

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
                _sort_lines(got, reply, telem)
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
            _sort_lines(self._pump(), reply, telem)
            time.sleep(0.004)
        return reply, telem


# ── Канал поверх модели прошивки: виртуальные часы вместо COM-порта ────────────
def _load_simulator():
    """Импортирует FirmwareSimulator из пакета sonar_gui.

    Ни sonar_gui/__init__.py, ни protocol.py, ни simulator.py не тянут Qt,
    поэтому обычного импорта пакета достаточно — PySide6 для --sim не нужен.
    Модуль protocol при этом загрузится вторым экземпляром (первый — по пути,
    как P): это чистые функции и константы, состояния в них нет.
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from sonar_gui.simulator import FirmwareSimulator
    return FirmwareSimulator


class SimLink:
    """Тот же интерфейс, что у Link, но за ним модель прошивки и виртуальные часы.

    Ни COM-порта, ни sleep: exchange/observe/pause крутят модель шагом в 1 мс —
    это период главного цикла прошивки (POLL_FREQ_HZ = 1 кГц), поэтому счётчик
    периода op=, задержка точки скана и событие om= отсчитываются в тех же
    единицах, что на плате, и кадры телеметрии появляются ровно там же.
    Время меряется в целых миллисекундах, так что прогон детерминирован —
    результат не зависит ни от загрузки ПК, ни от разрешения системных часов.
    """

    TICK_MS = 1

    def __init__(self):
        self.sim = _load_simulator()()
        self._t_ms = 0
        # Строки, которые плата шлёт сразу после сброса (диагностика энкодера)
        self._inbox: list[str] = list(self.sim.boot_lines())

    def close(self):
        pass

    def now(self) -> float:
        """Модельное время, с (замена time.monotonic() для раннера)."""
        return self._t_ms / 1000.0

    def pause(self, secs: float) -> None:
        self._advance(_ms(secs))

    def _advance(self, ms: int) -> list[str]:
        """Прокручивает модель на ms и отдаёт строки, выданные ею за это время."""
        out: list[str] = []
        for _ in range(ms // self.TICK_MS):
            self._t_ms += self.TICK_MS
            self.sim.tick(float(self.TICK_MS))
            line = self.sim.telemetry_tick(float(self.TICK_MS))
            if line:
                out.append(line)
        return out

    def exchange(self, cmd: str, settle: float = 0.30, idle: float = 0.06):
        """Отправляет cmd и слушает settle модельных секунд — как Link.exchange."""
        self._inbox = []            # аналог reset_input_buffer() на реальном порту
        reply: list[str] = []
        telem: list[str] = []
        # Терминатор дописывается так же, как его пишет в порт Link.exchange:
        # без CR+LF прошивка команду не соберёт и оставит в приёмном кольце
        # (line_reader.c:39-46), а модель теперь ведёт себя так же.
        _sort_lines(self.sim.handle_command(cmd + "\r\n"), reply, telem)

        idle_ms = _ms(idle)
        t_end = self._t_ms + _ms(settle)
        last = self._t_ms
        while self._t_ms < t_end:
            got = self._advance(self.TICK_MS)
            if got:
                last = self._t_ms
                _sort_lines(got, reply, telem)
            elif reply and (self._t_ms - last) >= idle_ms:
                break               # ответ есть, линия молчит — как ранний выход Link
        return reply, telem

    def observe(self, secs: float):
        reply: list[str] = []
        telem: list[str] = []
        _sort_lines(self._inbox + self._advance(_ms(secs)), reply, telem)
        self._inbox = []
        return reply, telem


def _ms(secs: float) -> int:
    """Секунды → целые миллисекунды модельных часов."""
    return int(round(secs * 1000.0))


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
        elif callable(w):
            # Элемент-проверка: строка разбирается по смыслу, а не по шаблону
            ok, reason = w(got)
            if not ok:
                return _fail(reason)
        elif w != got:
            return _fail(f"ждали {w!r}, пришло {got!r}")
    return True, ""


def check_enc_ok(line: str) -> tuple[bool, str]:
    """Отчёт стартовой диагностики энкодера — по критериям самой прошивки.

    main.c EncDiag_Finish/EncDiag_Report: вердикт «пройдено» ставится при
    ok_cnt >= ENCODER_DIAG_MIN_OK из ENCODER_DIAG_SAMPLES чтений и разбросе
    не больше ENCODER_DIAG_MAX_SPREAD_DEG, а сам разброс печатается как
    измерен (%.3f). Требовать буквального «n=16/16 spread=0.000» нельзя:
    такую строку исправная плата не обязана давать вовсе — один отсчёт
    17-битного энкодера это 360/131072 ≈ 0.0027°, и шум младшего бита уже
    даёт spread=0.003, а порог прошивки допускает 12 валидных чтений из 16.
    Проверяем ровно то, что прошивка обещает: формат строки и попадание
    чисел в её же критерий.
    """
    m = RE_ENC_OK_FMT.fullmatch(line)
    if not m:
        return _fail(f"{line!r} не по формату enc:ok n=…/… spread=… pos=…")
    ok_cnt, samples = int(m.group(1)), int(m.group(2))
    spread, pos = float(m.group(3)), float(m.group(4))
    if samples != P.ENCODER_DIAG_SAMPLES:
        return _fail(f"чтений {samples}, прошивка делает {P.ENCODER_DIAG_SAMPLES}")
    if not (P.ENCODER_DIAG_MIN_OK <= ok_cnt <= samples):
        return _fail(f"валидных {ok_cnt}/{samples}, порог прошивки "
                     f"{P.ENCODER_DIAG_MIN_OK}")
    if spread > P.ENCODER_DIAG_MAX_SPREAD_DEG:
        return _fail(f"разброс {spread}° больше порога прошивки "
                     f"{P.ENCODER_DIAG_MAX_SPREAD_DEG}°")
    if not 0.0 <= pos <= 360.0:
        return _fail(f"pos={pos} вне диапазона энкодера [0,360]")
    return True, ""


def tmc_reply(exact: str) -> Callable:
    """Ответ команды, которую исполняет драйвер TMC2209.

    tmc2209_motor_set_current()/_set_microsteps() возвращают -1 ещё до
    разбора значения, если драйвер не ответил по UART (s_tmc_ready == 0), и
    прошивка печатает err:not ready (main.c:1536-1539, 1574-1578); ошибка
    обмена даёт err:apply failed. Поэтому на плате с неподключённым или
    молчащим TMC2209 недостижим не только ok:, но и err:bad arg — ждать от
    неё точной строки нельзя. Против модели (ready всегда 1) эта поблажка не
    нужна и не применяется, см. run_all().
    """
    allowed = (exact, "err:not ready", "err:apply failed")

    def _c(reply: list[str]):
        if len(reply) != 1:
            return _fail(f"ждали одну строку из {list(allowed)}, пришло {reply}")
        if reply[0] not in allowed:
            return _fail(f"ждали {exact!r} или отказ неготового TMC2209, "
                         f"пришло {reply[0]!r}")
        return True, ""
    return _c


def check_mcfg(run: int, hold: int, ms: int) -> Callable:
    """mcfg показывает принятые токи и микрошаг.

    ready=0 значит, что драйвер по UART не ответил и предыдущие irun/ihold/
    mstep вернули err:not ready (tmc2209_motor.c:445,456) — применять им было
    нечего, поэтому сверять значения в этом случае не с чем; проверяется
    только то, что строка разобрана и режим управления тот же.
    """
    def _c(reply: list[str]):
        if len(reply) != 1:
            return _fail(f"mcfg: ждали 1 строку, пришло {reply}")
        m = P.parse_mcfg(reply[0])
        if not m:
            return _fail(f"mcfg не разобран: {reply[0]!r}")
        if m.get("mode") != "STEP_DIR":
            return _fail(f"mode={m.get('mode')} (ждали STEP_DIR)")
        if m.get("ready") not in (0, 1):
            return _fail(f"ready={m.get('ready')} (ждали 0/1)")
        if m.get("ready") == 0:
            return True, ""     # драйвер не отвечает — значения не применялись
        for k, v in (("run", run), ("hold", hold), ("microsteps", ms)):
            if m.get(k) != v:
                return _fail(f"{k}={m.get(k)} (ждали {v})")
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
    link: Link | SimLink
    quick: bool = False
    sim: bool = False            # прогон против модели: доступны проверки без железа
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
        """Ждёт, пока cp телеметрии подойдёт к target (нужна включённая телеметрия).

        Сходимость проверяется по кольцу: cp=359.9 и target=0 — одна точка,
        расхождение считается через кратчайший путь, как в прошивке.
        """
        target = P.wrap360(target)
        name = f"движение к {target:g}° (cp→{target:g}±{tol:g})"
        t_end = self.link.now() + (timeout if not self.quick else max(timeout, 1.5))
        last = None
        while self.link.now() < t_end:
            _, telem = self.link.observe(0.1)
            for ln in telem:
                d = P.parse_telemetry(ln)
                if d and "cp" in d:
                    last = d["cp"]
                    if abs(P.wrap180(last - target)) <= tol:
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
        self.link.pause(secs if not self.quick else min(secs, 0.2))

    # ── Событийная телеметрия (om=) и поведение скана во времени ───────────
    def evt_test(self, cmd: str, expect, target: float, name: str,
                 ev_count: int = 1, secs: float = 1.2, tol: float = 2.0):
        """Проверяет кадры ev:1 после команды cmd.

        Команда отправляется с коротким settle, а телеметрия собирается за
        последующие secs — иначе событийный кадр утонул бы внутри exchange.
        ev_count=1: ровно один кадр по приходу в цель, с позицией у target;
        ev_count=0: кадров быть не должно (om=0 или цель не менялась).
        """
        reply, telem = self.link.exchange(cmd, settle=0.05)
        _, more = self.link.observe(secs)
        telem += more
        ok, reason = evaluate(expect, reply)
        if ok:
            evs = [ln for ln in telem if ln.endswith(",ev:1")]
            if len(evs) != ev_count:
                ok, reason = _fail(
                    f"кадров ev:1 {len(evs)}, ждали {ev_count}: {telem[:4]}")
            elif ev_count == 1:
                d = P.parse_telemetry(evs[0])
                if not d or "cp" not in d:
                    ok, reason = _fail(f"кадр ev:1 не разобран: {evs[0]!r}")
                elif abs(P.wrap180(d["cp"] - P.wrap360(target))) > tol:
                    ok, reason = _fail(
                        f"cp={d['cp']} (ждали {P.wrap360(target):g}±{tol:g}): {evs[0]!r}")
        self._record(name, cmd, ok, reason)

    def wait_at_point(self, name: str, tol: float = 0.6, timeout: float = 3.0):
        """Ждёт, пока вал реально встанет в текущую уставку (|pe| ≤ tol).

        Предусловие для проверок «скан заморожен»: пока вал едет к первой
        точке, поле tp постоянно само по себе — и «точка не менялась» было
        бы верно даже при полностью сломанной заморозке. Нужна телеметрия
        debug=1: ошибка до уставки видна только в поле pe.
        """
        t_end = self.link.now() + timeout
        last = None
        while self.link.now() < t_end:
            _, telem = self.link.observe(0.05)
            for ln in telem:
                d = P.parse_telemetry(ln)
                if d and "pe" in d:
                    last = d["pe"]
            if last is not None and abs(last) <= tol:
                self._record(name, "<pe→0>", True, "")
                return True
        self._record(name, "<pe→0>", False,
                     f"вал не встал в точку: pe={last}")
        return False

    def scan_points(self, name: str, secs: float, want: int = 1,
                    frozen: bool = False, at_point_tol: float = 2.0):
        """Считает, сколько РАЗНЫХ точек скана (поле tp) прошло за secs.

        want — минимум разных точек (проверка «скан идёт»), frozen — обратное:
        точка обязана остаться единственной (скан заморожен). Нужна включённая
        телеметрия debug=1: точка скана видна только в поле tp.

        В режиме frozen дополнительно требуется, чтобы вал всё окно стоял в
        этой точке (|pe| ≤ at_point_tol). Без этого условия проверка прошла
        бы вхолостую на вале, который ещё едет к первой точке: tp у него
        постоянен независимо от того, работает заморозка или нет. Допуск
        крупнее мёртвой зоны: на снятом удержании вал на плате может слегка
        просесть, и это не провал заморозки.
        """
        _, telem = self.link.observe(secs)
        pts: list[float] = []
        pes: list[float] = []
        for ln in telem:
            d = P.parse_telemetry(ln)
            if not d:
                continue
            if "pe" in d:
                pes.append(d["pe"])
            if "tp" in d and (not pts or abs(P.wrap180(d["tp"] - pts[-1])) > 0.01):
                pts.append(d["tp"])
        if not telem:
            ok, reason = _fail("телеметрии не было — проверить нечего")
        elif frozen:
            worst = max((abs(p) for p in pes), default=None)
            if worst is None:
                ok, reason = _fail("нет поля pe — нужна телеметрия debug=1")
            elif worst > at_point_tol:
                ok, reason = _fail(
                    f"вал не стоял в точке всё окно (|pe| до {worst:.2f}°): "
                    "заморозку проверять не на чем")
            else:
                ok = len(pts) == 1
                reason = "" if ok else f"точка менялась: {pts[:6]}"
        else:
            ok = len(pts) >= want
            reason = "" if ok else f"точек {len(pts)} (ждали ≥{want}): {pts[:6]}"
        self._record(name, f"<{secs:.1f}s stream>", ok, reason)


def run_all(link: Link | SimLink, quick: bool, sim: bool = False) -> Runner:
    r = Runner(link, quick=quick, sim=sim)

    def tmc(exact: str):
        """Ожидание для команды, которую исполняет драйвер TMC2209.

        Против модели (ready всегда 1) сверяется точная строка — поблажка
        туда не проникает. На плате драйвер может молчать по UART, и тогда
        прошивка законно отвечает err:not ready вместо ok: И вместо
        err:bad arg: проверку ready она делает раньше разбора значения
        (tmc2209_motor.c:445,456). См. tmc_reply().
        """
        return exact if r.sim else tmc_reply(exact)

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
    r.t("t=0", "ok:t=0.00")
    # Координата кольцевая: угол вне [0,360) приводится к диапазону, а не
    # отвергается — эхо ok:t= показывает приведённое значение.
    r.t("t=-45.5", "ok:t=314.50", name="t=-45.5 → приводится к 314.50")
    r.t("t=370", "ok:t=10.00", name="t=370 → приводится к 10.00")
    r.t("t=720", "ok:t=0.00", name="t=720 → приводится к 0.00")
    r.t("t=360", "ok:t=0.00", name="t=360 → приводится к 0.00")
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

    r.phase("Источник телеметрии om= и запрос om")
    r.t("om=1", "ok:om=1")
    # Запрос без аргумента отвечает состоянием и БЕЗ префикса ok: — как sync
    r.t("om", "om=1", name="om → запрос режима (om=1)")
    r.t("om=2", "ok:om=2")
    r.t("om=0", "ok:om=0")
    r.t("om", "om=0", name="om → запрос отражает om=0")
    r.t("om=3", SILENCE, name="om=3 → молчание (вне 0..2)")
    r.t("om=-1", SILENCE, name="om=-1 → молчание")
    r.t("om=abc", SILENCE, name="om=abc → молчание")
    # Строку сборщик отдаёт как есть, Cmd_Parse сравнивает её целиком —
    # лишний пробел делает из запроса неизвестную команду (как у «sync »)
    r.t("om ", ERR_UNKNOWN, name="«om » с пробелом → err:unknown")

    r.phase("Удержание вала hold= и запрос hold")
    r.t("hold=0", "ok:hold=0")
    r.t("hold", "hold=0", name="hold → запрос удержания (hold=0)")
    r.t("hold=1", "ok:hold=1")
    r.t("hold", "hold=1", name="hold → запрос отражает hold=1")
    r.t("hold=2", SILENCE, name="hold=2 → молчание (только 0/1)")
    r.t("hold=-1", SILENCE, name="hold=-1 → молчание")
    r.t("hold=abc", SILENCE, name="hold=abc → молчание")
    r.t("hold ", ERR_UNKNOWN, name="«hold » с пробелом → err:unknown")

    r.phase("Неразбираемый аргумент → молчание прошивки")
    r.t("t=abc", SILENCE, name="t=abc → молчание")
    r.t("t=inf", SILENCE, name="t=inf → молчание (не конечное)")
    r.t("kp=xyz", SILENCE, name="kp=xyz → молчание")
    r.t("v=fast", SILENCE, name="v=fast → молчание")
    r.t("op=abc", SILENCE, name="op=abc → молчание")

    r.phase("Токи и микрошаг (мотор стоит)")
    r.t("stop", "ok:stop")
    r.t("irun 800", tmc("ok:irun=800"))
    r.t("irun 0", tmc("ok:irun=0"))
    r.t("irun 3000", tmc("ok:irun=3000"))
    r.t("irun 3001", SILENCE, name="irun 3001 → молчание (>3000)")
    r.t("ihold 350", tmc("ok:ihold=350"))
    r.t("icur 700 350", tmc("ok:icur=700,350"))
    r.t("icur 700", SILENCE, name="icur 700 → молчание (нет 2-го)")
    r.t("mstep 16", tmc("ok:mstep=16"))
    r.t("mstep 256", tmc("ok:mstep=256"))
    r.t("mstep 7", tmc(ERR_MSTEP))
    r.t("mstep 0", tmc(ERR_MSTEP))
    r.t("mstep -1", SILENCE, name="mstep -1 → молчание")
    # Границы аргумента mstep две, и они разные: набор 1/2/4…256 проверяет
    # драйвер (err:bad arg), а uint16 — парсер, ещё до обработчика
    # (cmd_parser.c: `v < 0 || v > 65535` → Cmd_Parse вернул 0, ответа нет).
    r.t(f"mstep {P.MSTEP_ARG_MAX}", tmc(ERR_MSTEP),
        name="mstep 65535 → err:bad arg (в uint16 влезает)")
    r.t(f"mstep {P.MSTEP_ARG_MAX + 1}", SILENCE,
        name="mstep 65536 → молчание (>65535)")
    r.t("mstep 65537", SILENCE,
        name="mstep 65537 → молчание (усечение дало бы ok:mstep=1)")
    r.t("mstep 66048", SILENCE,
        name="mstep 66048 → молчание (усечение дало бы 512)")

    r.phase("mcfg отражает состояние")
    # Смысл проверки одинаков на плате и в --sim: mcfg обязан показать именно
    # те токи и микрошаг, что были приняты. Отличается только поле ready —
    # у модели живого TMC2209 нет, оно всегда 1, поэтому ветки err:not ready
    # (драйвер не ответил по UART) в --sim непроверяемы; check_mcfg это учитывает.
    r.t("irun 750", tmc("ok:irun=750"))
    r.t("ihold 250", tmc("ok:ihold=250"))
    r.t("mstep 32", tmc("ok:mstep=32"))
    r.t("mcfg", check_mcfg(750, 250, 32), name="mcfg = run750 hold250 ms32 ready1")

    r.phase("Скан: сектор / бесконечный / ошибки")
    r.t("scan=0,90,10,100", "ok:scan=0.00,90.00,10.00,100")
    r.t("stop", "ok:stop")
    r.t("scan=0,+,5,50", "ok:scan=0.00,+,5.00,50")
    r.t("stop", "ok:stop")
    r.t("scan=0,-,5,50", "ok:scan=0.00,-,5.00,50")
    r.t("stop", "ok:stop")
    # end отсчитывается от start по кольцу: end < start — сектор через ноль
    r.t("scan=350,10,5,100", "ok:scan=350.00,10.00,5.00,100",
        name="scan через ноль (350→10, сектор 20°)")
    r.t("stop", "ok:stop")
    r.t("scan=-45,45,2,50", "ok:scan=315.00,45.00,2.00,50",
        name="scan ±45 → сектор 315→45 через ноль")
    r.t("stop", "ok:stop")
    r.t("scan=90,90,5,100", ERR_SCAN, name="scan start==end → err:scan")
    r.t("scan=0,360,5,100", ERR_SCAN, name="scan полный круг → err:scan")
    r.t("scan=0,90,0,100", ERR_SCAN, name="scan step=0 → err:scan")
    r.t("scan=0,90,5,0", ERR_SCAN, name="scan delay=0 → err:scan")
    r.t("scan=0,90,5", SILENCE, name="scan 3 поля → молчание")
    r.t("scan=abc,90,5,100", SILENCE, name="scan нечисло → молчание")

    r.phase("Синхронизация скана sync= / sync")
    r.t("sync", RE_SYNC, name="sync → статус (режим, пины, счётчик)")
    r.t("sync=1", "ok:sync=1")
    r.t("sync=2", "ok:sync=2")
    r.t("sync=0", "ok:sync=0")
    r.t("sync=3", SILENCE, name="sync=3 → молчание (вне 0..2)")
    r.t("sync=-1", SILENCE, name="sync=-1 → молчание")
    r.t("sync=abc", SILENCE, name="sync=abc → молчание")

    r.phase("Скан во времени: тайм-аут sync=2 и пауза hold=")
    r.t("en", "ok:en")
    r.t("debug=1", "ok:debug=1")   # точка скана видна только в поле tp
    r.t("op=20", "ok:op=20")
    r.t("sync=2", "ok:sync=2")
    r.t("scan=0,30,10,100", "ok:scan=0.00,30.00,10.00,100")
    r.scan_points("sync=2: точки сменяются по тайм-ауту delay", secs=1.0, want=3)
    r.t("stop", "ok:stop")
    # hold=0 приходит, когда вал уже стоит в первой точке и идёт отсчёт delay.
    # Приход в точку проверяется отдельным шагом: без него «точка не менялась»
    # было бы верно и для вала, ещё едущего к ней, — то есть проверка прошла
    # бы вхолостую даже при полностью сломанной заморозке.
    # Окно наблюдения заведомо длиннее остатка delay: незамороженный таймер за
    # это время обязан увести скан дальше — то есть плата «проехала» бы точку
    # с обесточенными обмотками, не сдвинув вал.
    r.t("scan=0,30,10,500", "ok:scan=0.00,30.00,10.00,500")
    r.wait_at_point("вал встал в первую точку скана (пошёл отсчёт delay)")
    # Короткий settle: слушать поток начинаем сразу, иначе незамороженный
    # таймер успел бы сменить точку внутри exchange — вне окна наблюдения.
    r.t("hold=0", "ok:hold=0", settle=0.05)
    r.scan_points("hold=0: отсчёт задержки скана заморожен", secs=1.2, frozen=True)
    r.t("hold=1", "ok:hold=1")
    r.scan_points("hold=1: скан продолжился с той же уставки", secs=1.2, want=2)
    r.t("stop", "ok:stop")
    if r.sim:
        # Без импульсов на SYNC_IN режим sync=1 обязан держать скан на первой
        # точке. На живой плате вход ловит наводки (ложный фронт уводит скан
        # дальше), поэтому проверка имеет смысл только против модели.
        r.t("sync=1", "ok:sync=1")
        r.t("scan=0,30,10,100", "ok:scan=0.00,30.00,10.00,100")
        r.wait_at_point("sync=1: вал встал в первую точку скана")
        r.scan_points("sync=1: без фронта SYNC_IN скан стоит", secs=1.0, frozen=True)
        r.t("stop", "ok:stop")
    r.t("sync=0", "ok:sync=0")
    r.t("op=0", "ok:op=0")
    r.t("debug=0", "ok:debug=0")

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
    # Проверяется контракт команды: на стоящем вале приходит ok:diag и отдельный
    # отчёт enc:ok в формате прошивки. У модели энкодер идеальный (спред 0.000,
    # 16/16 чтений) и отчёт приходит сразу, а не через ~30 мс, — ветка err:enc
    # в --sim недостижима, её проверяет только прогон на плате.
    r.t("stop", "ok:stop")
    r.pause(0.05)
    r.t("diag", ["ok:diag", check_enc_ok], name="diag idle → ok:diag + enc:ok", settle=0.45)

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

    r.phase("Событийная телеметрия om= — кадр ev:1 по приходу в цель")
    r.t("en", "ok:en")
    r.t("a=0", "ok:a=0.0")
    r.t("v=1200", "ok:v=1200.0")
    r.t("t=0", "ok:t=0.00")
    r.t("om=1", "ok:om=1")
    r.evt_test("t=90", "ok:t=90.00", 90.0,
               name="om=1 (op=0): ровно один кадр ev:1 в цели 90°")
    r.evt_test("t=180", "ok:t=180.00", 180.0,
               name="om=1: новая цель — новый кадр ev:1")
    r.evt_test("kp=0.025", "ok:kp=0.0250", 180.0, ev_count=0,
               name="om=1: цель не менялась — второго кадра нет")
    r.t("om=2", "ok:om=2")
    r.t("op=20", "ok:op=20")
    r.evt_test("t=270", "ok:t=270.00", 270.0,
               name="om=2: периодический поток и ровно один ev:1")
    r.t("op=0", "ok:op=0")
    r.t("om=0", "ok:om=0")
    r.evt_test("t=0", "ok:t=0.00", 0.0, ev_count=0,
               name="om=0: событийных кадров нет")

    r.phase("Замкнутый контур — реальное движение")
    r.t("en", "ok:en")
    r.t("a=0", "ok:a=0.0")
    r.t("v=1200", "ok:v=1200.0")
    r.t("op=20", "ok:op=20")
    r.t("t=0", "ok:t=0.00")
    r.converge(0.0)
    r.t("t=90", "ok:t=90.00")
    r.converge(90.0)
    r.t("t=-90", "ok:t=270.00", name="t=-90 → приводится к 270.00")
    r.converge(270.0)
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
    r.t(f"irun {d.irun}", tmc(f"ok:irun={d.irun}"))
    r.t(f"ihold {d.ihold}", tmc(f"ok:ihold={d.ihold}"))
    r.t(f"mstep {d.microsteps}", tmc(f"ok:mstep={d.microsteps}"))
    r.t(f"op={d.op_ms}", f"ok:op={d.op_ms}")
    r.t("om=0", "ok:om=0")
    r.t("hold=1", "ok:hold=1")
    r.t("sync=0", "ok:sync=0")
    return r


# ── Порт ────────────────────────────────────────────────────────────────────────
def find_port() -> str | None:
    """WCH-Link VCP (1a86) / STM32 (0483) / CH340 / CP210x / FTDI."""
    if list_ports is None:
        return None
    for p in list_ports.comports():
        hwid = (p.hwid or "").lower()
        desc = (p.description or "").lower()
        if any(v in hwid for v in ("1a86", "0483", "ch340", "cp210", "0403")) or "stm" in desc:
            return p.device
    return None


def print_ports():
    if list_ports is None:
        print("Нужен pyserial: pip install pyserial")
        return
    ports = list(list_ports.comports())
    if not ports:
        print("COM-портов не найдено.")
        return
    for p in ports:
        print(f"  {p.device:8}  {p.description or '?'}  [{p.hwid or '?'}]")


def main() -> int:
    # Консоль Windows по умолчанию в cp866/cp1251: рамки ── и ═ в неё не
    # кодируются, и прогон падал бы на первом же заголовке фазы.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, OSError):
            pass

    ap = argparse.ArgumentParser(description="Конформанс-тест протокола FW_SonarMotorDriver")
    ap.add_argument("port", nargs="?", help="COM-порт (по умолчанию — автопоиск)")
    ap.add_argument("--list", action="store_true", help="показать порты и выйти")
    ap.add_argument("--quick", action="store_true", help="без длинных пауз скана/движения")
    ap.add_argument("--sim", action="store_true",
                    help="прогон против Python-модели прошивки, без порта и без ожиданий")
    args = ap.parse_args()

    if args.list:
        print_ports()
        return 0

    if args.sim:
        link: Link | SimLink = SimLink()
        print("Модель прошивки: sonar_gui/simulator.py, виртуальные часы (плата не нужна)")
    else:
        if serial is None:
            print("Нужен pyserial: pip install pyserial "
                  "(или прогон без железа: python protocol_test.py --sim)",
                  file=sys.stderr)
            return 2
        port = args.port or find_port()
        if not port:
            print("Порт не найден. Доступные:", file=sys.stderr)
            print_ports()
            print("\nУкажите явно: python protocol_test.py COM25", file=sys.stderr)
            print("Без платы: python protocol_test.py --sim", file=sys.stderr)
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

    t0 = time.perf_counter()
    try:
        r = run_all(link, quick=args.quick, sim=args.sim)
    except KeyboardInterrupt:
        print("\nПрервано.", file=sys.stderr)
        return 130
    finally:
        link.close()
    wall = time.perf_counter() - t0

    total = r.passed + r.failed
    print("\n" + "═" * 52)
    timing = f"за {wall:.2f} с"
    if args.sim:
        timing += f" (модельного времени {link.now():.1f} с)"
    print(f"ИТОГО: {r.passed}/{total} пройдено, {r.failed} провалено, {timing}")
    if r.fails:
        print("\nПровалы:")
        for name, reason in r.fails:
            print(f"  - {name}: {reason}")
    print("═" * 52)
    return 0 if r.failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
