"""FirmwareSimulator — Python-модель прошивки FW_SonarMotorDriver.

Чистая логика без Qt: разбирает те же команды и отдаёт побитово те же строки
ответов/телеметрии, что и железо (иначе разбор в GUI не проверился бы).
Обёртку с таймерами для GUI даёт transport/sim_transport.py; конформанс-тест
протокола гоняет эту же модель на виртуальных часах (protocol_test.py --sim).

Поведение воспроизводит прошивку:
- разбор аргументов повторяет Cmd_Parse() (lib/sonar_proto/src/cmd_parser.c):
  число читается префиксом, как strtof/strtol, «хвост» после числа парсер не
  замечает ("op=10x" → ok:op=10), а нечитаемый аргумент даёт «молчание»
  (пустой список строк). Вещественные аргументы разбираются со всеми
  повадками strtof: шестнадцатеричная форма ("t=0x10" → ok:t=16.00),
  диапазон float (за его пределами аргумент гибнет на !isfinite) и isspace
  в ведущих пробелах — подробности у _parse_float;
- сборка строк — как в line_reader.c: команда выполняется только после
  терминатора CR/LF, а незавершённый хвост куска ждёт его в приёмнике
  (line_reader.c:39-46); пустые строки пропускаются, но разбор на них не
  прекращается, поэтому в одном куске может приехать несколько команд, и
  все они будут выполнены по порядку;
- порядок внутри тика — как в главном цикле main.c: движение, затем скан,
  затем телеметрия;
- система координат кольцевая, один оборот: cur_deg и target_deg всегда
  в [0,360), ошибка считается по кратчайшему пути (≤180°) — как в прошивке
  (main.c, shortest_path_err + wrap360), накопителя оборотов нет;
- контур положения — тот же ПИД, что на плате (pid.c + main.c:976-978):
  out = kp·e + ki·∫e·dt + kd·de/dt, выход ограничен границами насыщения из
  состояния регулятора (на старте ±MAX_DEG_PER_TICK, пересчёт — только в
  обработчике v=, см. pid_out_max),
  интеграл в насыщении не копится (anti-windup), сброс — ровно там же, где
  прошивка зовёт PID_Reset()/Control_Reset(); скорость с выхода ПИД дальше
  режется профилем v=/a=, поэтому kp/ki/kd влияют на время схождения;
- позиция cur_deg плавно идёт к target_deg с ограничением скорости (v=)
  и ускорения (a=, 0 = мгновенно) — как профиль движения прошивки;
- вход в мёртвую зону — с доводочным шагом (snap, main.c:983-996): вал
  садится точно в цель, а событие «приехали» взводится следующим тиком;
  пока серия шагов не отработана, вал считается движущимся (main.c:1006-1009),
  поэтому mstep и diag, пришедшие в это окно, получают err:busy;
- непрерывное вращение t=+/t=-;
- сектор (зигзаг, в т.ч. пересекающий ноль) и бесконечный скан с задержкой
  delay в каждой точке;
- синхронизация sync=0/1/2 и запрос sync; физического пина SYNC_IN в модели
  нет, поэтому в режиме sync=1 скан честно стоит на точке (как прошивка без
  импульсов), в sync=2 — уходит дальше по тайм-ауту delay;
- телеметрия по периоду op= и/или по приходу в цель (om=); событийный кадр
  помечен полем ev:1 и выдаётся ровно один раз на цель;
- hold= — тихая пауза: контур и таймер скана заморожены, цель и точка скана
  сохраняются;
- запросы состояния без аргумента (sync, om, hold) отвечают одной строкой
  данных без префикса ok: — «sync=... in=... out=... n=...», «om=N»,
  «hold=N» (main.c CMD_GET_SYNC / CMD_GET_OUTPUT_MODE / CMD_GET_HOLD).

Чего в модели нет (и почему это не мешает сверке протокола):
- очереди UART: прошивка придерживает кадр телеметрии, пока занят передатчик
  (Telemetry_Tick: `if (UART_TxPending()) return;`), и считает потери в drp —
  у модели кадр уходит всегда, поэтому drp растёт только на вытесненном
  событийном кадре и в обычном прогоне остаётся нулём;
- шагового генератора: импульсы STEP выдаются мгновенно и без квантования по
  микрошагу (у прошивки ApplyVelocity округляет скорость до целых шагов в
  секунду, а доводочный шаг — до целого числа микрошагов), поэтому mstep=
  на точность модели не влияет. Длительность доводочной серии модель берёт
  не из счётчика импульсов, а из её устройства: прошивка растягивает серию
  ровно на один период опроса (arr = TICKS_PER_POLL / n, tmc2209_motor.c:283),
  и модель держит окно err:busy ровно один свой тик — на тике GUI в 10 мс
  окно выйдет во столько же раз длиннее реального;
- защиты от блокировки вала (Stall_Tick): нагрузки на вал у модели нет, ec
  всегда 0, строка err:stall и «залипший» ERR_STALL недостижимы;
- фильтра выбросов и режима Open-Loop: энкодер модели идеален, поэтому поля
  of/ec всегда нули, m: всегда cl, а ветки коастинга и перехода CL→OL при
  молчащем энкодере не проверяются;
- пинов SYNC_IN/SYNC_OUT и живого TMC2209: фронтов нет (in=0, n=0 — фронт
  вносится тестом вручную), драйвер всегда ready=1, поэтому ветки
  err:not ready / err:apply failed недостижимы;
- диагностика энкодера всегда успешна, и её отчёт приходит тем же ответом,
  а не отдельной строкой через ~30 мс, как на плате;
- сторожевого таймера и регистра RCC->CSR: boot-баннер модель отдаёт с
  фиксированными полями (см. boot_lines), rst=iwdg воспроизвести нельзя.
"""
from __future__ import annotations

import math
import re
import struct

from . import protocol as P

DEADBAND_DEG = 0.05                 # board.h PID_DEADBAND_DEG
TELEM_MODE_DEFAULT = 0              # board.h TELEMETRY_MODE_DEFAULT (om=)
HOLD_DEFAULT = 1                    # прошивка стартует с удержанием (g_hold = 1)
OUTPUT_PERIOD_MS_DEBUG_MIN = 20     # board.h OUTPUT_PERIOD_MS_DEBUG_MIN
SNAP_MIN_DEG = 0.02                 # main.c: порог доводочного шага в мёртвой зоне

# Такт контура прошивки: главный цикл идёт на POLL_FREQ_HZ = 1 кГц, и ПИД
# всегда считается с этим шагом (main.c: PID_Update(&g_pid, err, DT_S)).
# Выход ПИД — градусы ЗА ТАКТ, тогда как модель ведёт скорость в °/с, поэтому
# при переводе одного в другое фигурирует именно эта константа, а не dt тика
# модели (у GUI тик 10 мс — это десять тактов платы разом).
CONTROL_DT_S = 0.001                # main.c DT_S = 1 / POLL_FREQ_HZ

# Boot-баннер (main.c Boot_Banner, Docs/Protocol.md «Загрузочная
# последовательность»). Формат строки — байт-в-байт как у прошивки
# ("boot:rst=%s fw=%s"), а значения полей для модели выбраны так:
#  - rst=por: модель «включается» вместе с процессом, что равнозначно подаче
#    питания на плату (por — power-on, первая строка платы после включения);
#    остальные флаги взять неоткуда — ни сторожевого таймера, ни регистра
#    RCC->CSR у модели нет, так что rst=iwdg/sft/pin она не покажет никогда;
#  - fw: версия прошивки, поведение которой модель воспроизводит
#    (-DFW_VERSION в FW_SonarMotorDriver/platformio.ini), с суффиксом
#    реализации — как у прошивки-имитатора с её "1.0.0-sim". В логе хоста
#    сразу видно, кто ответил: плата, имитатор или эта модель.
BOOT_RST_FLAGS = "por"
FW_VERSION = "1.0.0-model"

# Источник выдачи телеметрии (om=N) — enum TelemSrc в main.c
TELEM_SRC_PERIOD = 0                # только период op=
TELEM_SRC_TARGET = 1                # только приход в цель
TELEM_SRC_BOTH = 2                  # оба источника
TELEM_MODE_VALUES = (TELEM_SRC_PERIOD, TELEM_SRC_TARGET, TELEM_SRC_BOTH)

# Источник перехода к следующей точке скана (sync=N) — board.h SyncAdv
SYNC_ADV_TIMER = 0
SYNC_ADV_EXT = 1
SYNC_ADV_EXT_TIMEOUT = 2

# Состояние скана — enum ScanState в main.c
SCAN_IDLE = 0
SCAN_MOVING = 1
SCAN_DELAY = 2


# ── Разбор чисел строго как strtof/strtol в cmd_parser.c ───────────────────
# C-функции читают числовой ПРЕФИКС и оставляют курсор на первом непрочитанном
# символе: "op=10x" прошивка принимает как op=10, "5e" — как 5, а "icur 7 3 9"
# — как icur 7,3. float()/int() Python на таких строках падают, из-за чего
# модель молчала бы там, где плата отвечает. Регулярки повторяют грамматику
# strtof/strtol в том виде, в каком её даёт newlib прошивки:
#  - ведущие пробелы обе функции снимают по isspace(), то есть вместе с \v и
#    \f (в firmware.elf у _strtod_l это явное сравнение с 9..13 и 32, у
#    _strtol_l — бит _S таблицы ctype); CR и LF до Cmd_Parse не доходят —
#    их снимает сборщик строки;
#  - strtof по C99 читает и шестнадцатеричную форму 0x1.8p3: в прошивку
#    слинкованы __gethex/__hexdig_fun, поэтому "t=0x10" плата выполняет как
#    t=16.00. Экспонента у этой формы двоичная (p), хвост без её цифр
#    ("0x1p") strtof откатывает, а "0x" без шестнадцатеричных цифр читается
#    как обычный ноль с курсором на 'x';
#  - strtol вызывается с основанием 10 и на 'x' останавливается, поэтому у
#    целых аргументов (op=/om=/sync=/hold=) формы 0x нет: "op=0x10" → op=0;
#  - inf/nan strtof читает, но прошивка отвергает их проверкой isfinite и
#    молчит; регулярка их не матчит — результат тот же.
_RE_HEX_FLOAT = (r"0[xX](?:[0-9a-fA-F]+(?:\.[0-9a-fA-F]*)?|\.[0-9a-fA-F]+)"
                 r"(?:[pP][+-]?[0-9]+)?")
_RE_DEC_FLOAT = r"(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?"
_RE_SPACE = r"[ \t\n\v\f\r]*"
_RE_FLOAT = re.compile(
    _RE_SPACE + r"[+-]?(?:" + _RE_HEX_FLOAT + r"|" + _RE_DEC_FLOAT + r")")
_RE_INT = re.compile(_RE_SPACE + r"[+-]?[0-9]+")

# Конец строки в приёмном потоке: CR либо LF (line_reader.c:27 — оба байта
# завершают строку, пара CR+LF считается одним концом).
_RE_EOL = re.compile(r"[\r\n]")


def _to_float32(v: float) -> float:
    """Округление double до float — как возврат из strtof (там (float)val).

    Разобранное число прошивка кладёт в поле float структуры Cmd_Result,
    поэтому за пределами диапазона float аргумент превращается в
    бесконечность и гибнет на !isfinite: "t=1e40" плата не выполняет, хотя
    в double это число живо.
    """
    try:
        return struct.unpack("<f", struct.pack("<f", v))[0]
    except OverflowError:           # больше DBL_MAX упаковать не выйдет
        return math.inf if v > 0.0 else -math.inf


def _parse_float(s: str, i: int = 0):
    """(значение, позиция за числом) либо None — как parse_float() прошивки."""
    m = _RE_FLOAT.match(s, i)
    if not m:
        return None
    text = m.group().strip()
    try:
        v = (float.fromhex(text) if "x" in text or "X" in text
             else float(text))
    except (ValueError, OverflowError):     # "0x1p9999" — вне диапазона double
        return None
    v = _to_float32(v)
    if not math.isfinite(v):        # "1e400" → inf, прошивка отвергает !isfinite
        return None
    return v, m.end()


def _parse_int(s: str, i: int = 0):
    """(значение, позиция за числом) либо None — как parse_int() прошивки."""
    m = _RE_INT.match(s, i)
    if not m:
        return None
    return int(m.group()), m.end()


def _skip_spaces(s: str, i: int) -> int:
    while i < len(s) and s[i] in " \t":
        i += 1
    return i


def _arg_float(s: str):
    """Аргумент-число целиком (t=, kp=, v=, …): хвост после числа игнорируется."""
    r = _parse_float(s)
    return None if r is None else r[0]


def _arg_int(s: str):
    """Аргумент-целое (op=, om=, debug=, sync=, hold=): хвост игнорируется."""
    r = _parse_int(s)
    return None if r is None else r[0]


class FirmwareSimulator:
    def __init__(self) -> None:
        d = P.DEFAULTS
        # Приёмное кольцо UART: незавершённый хвост куска ждёт здесь
        # терминатора, как байты в rx_ring между вызовами UART_ReadLine().
        self._rx = ""
        self.enabled = True
        self.hold = HOLD_DEFAULT
        self.ready = True
        self.mode = "cl"            # sim всегда замкнутый контур
        self.ec = 0
        self.drp = 0

        self.cur_deg = d.target_deg
        self.target_deg = d.target_deg
        self.last_u = 0.0

        self.cont_dir = 0           # 0 / +1 / -1 — непрерывное вращение

        # Скан. Зигзаг ведётся в координате сектора scan_u ∈ [0, scan_span]
        # (как g_scan_u в прошивке), поэтому сектор через ноль разворачивается
        # на краях так же, как обычный.
        self.scan_st = SCAN_IDLE    # IDLE / MOVING / DELAY — как g_scan_st
        self.scan_start = 0.0       # начало сектора, абсолютный угол [0,360)
        self.scan_span = 0.0        # протяжённость от start по возрастанию, (0,360)
        self.scan_u = 0.0           # текущая точка в координате сектора
        self.scan_step = 0.0
        self.scan_delay_ms = 0
        self.scan_inf = 0           # 0 / +1 / -1 — бесконечный скан и его знак
        self.scan_dir = 1
        self.scan_sp = 0.0          # текущая точка, абсолютный угол [0,360)
        self.scan_dwell_ms = 0.0

        # Синхронизация (sync=): физического SYNC_IN в модели нет — фронтов 0
        self.sync_mode = 0
        self.sync_edges = 0
        self.syncin_trig = 0
        self.sync_out = 0           # уровень SYNC_OUT: 1 = точка скана достигнута

        # ПИД-регулятор положения: коэффициенты и накопленное состояние
        # (PID_State в pid.h). kp/ki/kd меняются командами kp=/ki=/kd=.
        self.kp = d.kp
        self.ki = d.ki
        self.kd = d.kd
        self.pid_i = 0.0            # integral: накопленная ошибка, °·с
        self.pid_prev_err = 0.0     # prev_error: ошибка прошлого такта
        self.pid_init = 0           # initialized: на первом такте производной нет
        # Границы насыщения выхода (PID_State.output_min/max), градусы за такт.
        # Прошивка задаёт их статическим инициализатором от АППАРАТНОГО потолка
        # скорости (main.c:416, MAX_DEG_PER_TICK = MAX_SPEED_DEG_S/POLL_FREQ_HZ)
        # и пересчитывает только в обработчике v= (main.c:1519-1520). От
        # SPEED_DEFAULT_DEG_S стартовое значение не зависит: сегодня эти две
        # константы равны (board.h), но равенство ничем не закреплено, и брать
        # границы от vmax значило бы держаться за случайное совпадение.
        self.pid_out_max = P.MAX_SPEED_DEG_S * CONTROL_DT_S
        self.was_outside_db = 0     # g_was_outside_db: ждём доводочный шаг
        self.snap_busy = 0          # доводочная серия шагов ещё не отработана

        self.op_ms = d.op_ms
        self.debug = d.debug
        self.irun = d.irun
        self.ihold = d.ihold
        self.microsteps = d.microsteps

        # Выдача телеметрии: период op= (счётчик миллисекунд) и событие om=
        self.telem_mode = TELEM_MODE_DEFAULT
        self._telem_cnt = 0.0
        self._at_target = 0         # уровень «стоим в цели» (g_at_target)
        self._evt_armed = 1         # право на кадр, одно на цель (g_evt_armed)
        self._evt_pending = 0       # кадр взведён и ещё не выдан (g_telem_evt)
        self._evt_deg = 0.0         # снимок позиции на момент прихода
        self._evt_tp = 0.0          # снимок цели на момент прихода

        # Профиль движения (v= / a=), как в прошивке
        self.vmax = d.vmax          # предел скорости, °/с
        self.accel = d.accel        # предел ускорения, °/с² (0 = мгновенно)
        self.vel = 0.0              # текущая скорость модели, °/с (знаковая)

    # ── Стартовые сообщения ────────────────────────────────────────────────
    def boot_lines(self) -> list[str]:
        """Строки, которые прошивка шлёт при старте, в порядке их выдачи.

        1. Boot-баннер `boot:rst=<флаги> fw=<версия>` — первая строка платы
           после подъёма UART (main.c Boot_Banner). Значения у модели
           фиксированные, см. BOOT_RST_FLAGS / FW_VERSION.
        2. Отчёт стартовой диагностики энкодера: реальная прошивка перед
           разрешением движения делает серию чтений BiSS-C и отправляет
           enc:ok / err:enc. У модели диагностика всегда успешна — формат
           строки повторяет прошивку байт-в-байт.
        """
        return [
            f"boot:rst={BOOT_RST_FLAGS} fw={FW_VERSION}",
            f"enc:ok n=16/16 spread=0.000 pos={self.cur_deg:.2f}",
        ]

    # ── Состояние ──────────────────────────────────────────────────────────
    def _err(self) -> float:
        """Ошибка до цели по кратчайшему пути, (-180, 180] — как в прошивке."""
        return _wrap180(self.target_deg - self.cur_deg)

    @property
    def scan_active(self) -> bool:
        """Скан запущен — как условие g_scan_st != SCAN_IDLE в прошивке."""
        return self.scan_st != SCAN_IDLE

    def is_moving(self) -> bool:
        """Вал реально крутится — критерий tmc2209_motor_is_moving() прошивки.

        Именно он решает, принимать ли mstep. В паузе между точками скана вал
        стоит, поэтому запущенный скан сам по себе движением не считается;
        более широкое условие «занят» (для diag) даёт is_busy().

        Доводочная серия шагов тоже считается движением: прошивка отдаёт её
        таймеру (tmc2209_motor_move_steps → s_pulses_left > 0, main.c:1006-1009),
        и пока импульсы не отработаны, is_moving() истинно, хотя ошибка уже в
        мёртвой зоне — mstep и diag в этом окне получают err:busy. Серия
        растянута ровно на один период опроса (arr = TICKS_PER_POLL / n,
        tmc2209_motor.c:283), поэтому окно у модели одно — до следующего тика.
        """
        if not self.enabled or not self.hold:
            return False
        if self.cont_dir != 0:
            return True
        if self.snap_busy:
            return True
        return abs(self._err()) > DEADBAND_DEG

    def is_busy(self) -> bool:
        """Условие err:busy для diag (main.c CMD_DIAG): вращение, скан или ход."""
        return self.cont_dir != 0 or self.scan_st != SCAN_IDLE or self.is_moving()

    def scan_range(self):
        """Текущий сектор скана для подсветки на диаграмме, либо None.

        Возвращает (start, end) в абсолютных углах: у сектора через ноль
        end < start, отрисовка обязана идти от start в сторону возрастания.
        """
        if not self.scan_active or self.scan_inf:
            return None
        return (self.scan_start, _wrap360(self.scan_start + self.scan_span))

    # ── Служебное состояние контура (как одноимённые функции main.c) ────────
    def _pid_reset(self) -> None:
        """PID_Reset() из pid.c: интеграл, прошлая ошибка и флаг первого такта."""
        self.pid_i = 0.0
        self.pid_prev_err = 0.0
        self.pid_init = 0

    def _control_reset(self) -> None:
        """Control_Reset(): интеграл PID и рампа скорости — с нуля."""
        self._pid_reset()
        self.vel = 0.0

    def _pid_update(self, err: float, dt_s: float) -> float:
        """PID_Update() из pid.c. err — градусы, возврат — градусы за такт.

        Структура вычислений повторяет прошивку: интеграл копится по времени,
        производная считается по разности с прошлым тактом (на первом такте
        после сброса её нет), выход насыщается на ±output_max и при насыщении
        интеграл откатывается назад (anti-windup, pid.c:41-42).

        Отличий от платы два, и оба про числа, а не про структуру.

        Первое — dt берётся от тика модели, а не фиксированный DT_S. При тике
        1 мс (тесты, protocol_test --sim) это в точности DT_S, и такты идут
        тем же чередом, что на плате. При укрупнённом тике GUI (10 мс)
        совпадения уже нет: интеграл даёт тот же прирост, только если ошибка за
        интервал не менялась, а производная (err − prev_err)/dt считается по
        разности за весь тик и отличается от платы всегда, когда ошибка внутри
        тика шла не по прямой.

        Второе — разрядность: у прошивки и аргументы PID_Update(), и все поля
        PID_State объявлены float, а Python считает в double. Поэтому даже на
        такте 1 мс совпадение с платой не побитовое: числа расходятся в младших
        разрядах и накапливаются в интеграле. Для сверки протокола это неважно
        (углы в телеметрии печатаются с двумя знаками), но «такт в такт» здесь
        означает одинаковую траекторию, а не одинаковые биты.
        """
        self.pid_i += err * dt_s
        deriv = (err - self.pid_prev_err) / dt_s if self.pid_init else 0.0
        self.pid_prev_err = err
        self.pid_init = 1

        out = self.kp * err + self.ki * self.pid_i + self.kd * deriv

        # Насыщение на ±output_max: границы хранятся в состоянии регулятора и
        # меняются только обработчиком v= (см. pid_out_max в __init__).
        out_max = self.pid_out_max
        if out < -out_max:
            out = -out_max
        elif out > out_max:
            out = out_max

        if self.ki != 0.0 and (out == -out_max or out == out_max):
            self.pid_i -= err * dt_s
        return out

    def _target_reset_reached(self) -> None:
        """Новая цель — новое событие, даже если вал уже стоит в ней."""
        self._at_target = 0
        self._evt_armed = 1

    def _target_mark_reached(self) -> None:
        """Приход в цель: взводит событийный кадр телеметрии (om=1/2)."""
        if self._at_target:
            return
        self._at_target = 1
        if self.telem_mode == TELEM_SRC_PERIOD:
            return
        # Строго один кадр на одну цель: право выдаётся только сменой цели.
        if not self._evt_armed:
            return
        self._evt_armed = 0
        if self._evt_pending:       # прежний кадр не успел уйти — потеря видна в drp
            self.drp += 1
        self._evt_deg = self.cur_deg
        self._evt_tp = self.target_deg
        self._evt_pending = 1

    # ── Приём команды ──────────────────────────────────────────────────────
    def handle_command(self, chunk: str) -> list[str]:
        """Ответы на всё, что прошивка вычитает из chunk. Пустой — «молчание».

        Аргумент — КУСОК ПОТОКА, а не готовая строка: модель повторяет
        сборщик line_reader_extract() вместе с его приёмным кольцом.

        1. Команда выполняется только после терминатора. Без CR/LF сборщик
           возвращает 0 и оставляет хвост в кольце (line_reader.c:39-46:
           `if (!found_eol) { ... return 0; }`), поэтому "en" без конца
           строки ответа не даёт вовсе, а приезжает он позже — когда
           терминатор придёт следующим куском. Незавершённый хвост копится
           в self._rx между вызовами, как байты в кольце между тиками.
        2. Пара CR+LF — один конец строки (line_reader.c:30-33), и пустую
           строку сборщик наружу не отдаёт, но разбор на ней не прекращает:
           пропускает и ищет следующую в том же кольце (line_reader.c:66-69).
           Поэтому "\\r\\nen\\r\\n" даёт ok:en, а "en\\rdis\\r" — ok:en и
           ok:dis, в том же порядке.
        3. Пробелы по краям сборщик не снимает (line_reader.c:52-60 копирует
           все байты, пропуская только CR и LF), и Cmd_Parse сравнивает
           строку как есть, поэтому " en" и "   " командами не считаются и
           дают err:unknown.

        Отличие от платы одно, и оно про темп, а не про содержание: главный
        цикл вычитывает из кольца по одной строке за тик (main.c:1350-1356
        PollCommands), а модель отдаёт ответы на все готовые строки куска
        сразу. Порядок команд и ответов тот же; разъезжается только момент
        выдачи внутри одного тика.

        Чего модель не повторяет — ограничений на длину. Кольцо платы 128
        байт (board.h UART_RX_RING_SIZE): строка длиннее теряет по байту с
        головы (line_reader.c:41-45), а из того, что осталось, PollCommands
        берёт первые 63 символа (main.c:1352 `char line[64]`). Хвост в модели
        не ограничен ничем, поэтому команды длиннее кольца она разбирает
        целиком, а плата — покалеченными.
        """
        out: list[str] = []
        self._rx += chunk
        while True:
            m = _RE_EOL.search(self._rx)
            if m is None:               # терминатора нет — хвост ждёт в кольце
                return out
            end = m.end()
            if (self._rx[m.start()] == "\r" and end < len(self._rx)
                    and self._rx[end] == "\n"):
                end += 1                # CR+LF — один конец строки
            line, self._rx = self._rx[:m.start()], self._rx[end:]
            if line:                    # пустую строку сборщик пропускает
                out += self._handle_line(line)

    def _handle_line(self, c: str) -> list[str]:
        """Ответы на одну собранную строку.

        Порядок сравнений повторяет Cmd_Parse(): сначала команды целиком
        (en/dis/stop/t=+/t=-), затем префиксные с аргументом, в конце —
        err:unknown.
        """
        if c == "en":
            # Как CMD_ENABLE: удержание возвращается, цель привязывается к
            # текущей позиции вала (g_target_deg = Enc_Deg()), профиль с нуля.
            self.enabled = True
            self.hold = 1
            self.cont_dir = 0
            self._control_reset()
            self.target_deg = self.cur_deg
            self._target_reset_reached()
            return ["ok:en"]
        if c == "dis":
            # Как CMD_DISABLE: состояние скана и контура не трогаем — плата
            # просто снимает разрешение и останавливает мотор.
            self.enabled = False
            self.cont_dir = 0
            return ["ok:dis"]
        if c == "stop":
            self.cont_dir = 0
            self.scan_inf = 0
            self.scan_st = SCAN_IDLE
            self.snap_busy = 0      # как tmc2209_motor_stop(): серия обрывается
            self.target_deg = self.cur_deg
            self._control_reset()
            self._target_reset_reached()
            self.syncin_trig = 0
            self.sync_out = 0
            return ["ok:stop"]
        if c in ("t=+", "t=-"):
            # Как CMD_CONTINUOUS: джог отменяет и dis, и снятое удержание,
            # иначе вал не тронулся бы с места.
            self.cont_dir = 1 if c == "t=+" else -1
            self.scan_st = SCAN_IDLE
            self._pid_reset()       # как прошивка: PID_Reset, рампу не трогаем
            if not self.enabled or not self.hold:
                self.enabled = True
                self.hold = 1
            return [f"ok:t={c[2]}"]

        if c.startswith("t="):
            v = _arg_float(c[2:])
            if v is None:
                return []
            # Как прошивка: любой угол приводится к [0,360), эхо — приведённое
            # значение (t=370 -> ok:t=10.00, t=-30 -> ok:t=330.00). Скан при
            # этом не отменяется (CMD_SET_TARGET трогает только g_cont_dir):
            # вал идёт в заданный угол, а дальше скан ведёт его по своим точкам.
            self.target_deg = _wrap360(v)
            self.cont_dir = 0
            # Как прошивка: PID с чистым интегралом, а рампа скорости (vel)
            # сохраняется — смена цели на ходу продолжает разгон плавно.
            self._pid_reset()
            self._target_reset_reached()
            return [f"ok:t={self.target_deg:.2f}"]
        if c.startswith("kp="):
            v = _arg_float(c[3:])
            if v is None:
                return []
            self.kp = v
            return [f"ok:kp={v:.4f}"]
        if c.startswith("ki="):
            v = _arg_float(c[3:])
            if v is None:
                return []
            self.ki = v
            return [f"ok:ki={v:.4f}"]
        if c.startswith("kd="):
            v = _arg_float(c[3:])
            if v is None:
                return []
            self.kd = v
            return [f"ok:kd={v:.4f}"]
        if c.startswith("v="):
            v = _arg_float(c[2:])
            if v is None:
                return []
            if not P.speed_ok(v):
                return [f"err:bad arg (v={P.SPEED_MIN_DEG_S:g}..{P.MAX_SPEED_DEG_S:g})"]
            self.vmax = v
            # Единственное место, где прошивка двигает границы насыщения ПИД
            # (main.c:1519-1520): иначе clamp профиля обогнал бы регулятор.
            self.pid_out_max = v * CONTROL_DT_S
            return [f"ok:v={v:.1f}"]
        if c.startswith("a="):
            v = _arg_float(c[2:])
            if v is None:
                return []
            if not P.accel_ok(v):
                return [f"err:bad arg (a=0..{P.ACCEL_MAX_DEG_S2:g})"]
            self.accel = v
            return [f"ok:a={v:.1f}"]
        if c.startswith("op="):
            n = _arg_int(c[3:])
            if n is None or not P.op_ok(n):
                return []
            self.op_ms = n
            return [f"ok:op={n}"]
        if c.startswith("om="):
            n = _arg_int(c[3:])
            if n is None or n not in TELEM_MODE_VALUES:
                return []
            self.telem_mode = n
            # Событие, взведённое в прежнем режиме, отменяем: хост, только что
            # переключивший режим, ждёт кадр по своей следующей цели.
            self._evt_pending = 0
            return [f"ok:om={n}"]
        if c.startswith("debug="):
            n = _arg_int(c[6:])
            if n is None or n not in (0, 1):
                return []
            self.debug = n
            return [f"ok:debug={n}"]
        if c.startswith("scan="):
            return self._cmd_scan(c[5:])
        if c.startswith("sync="):
            n = _arg_int(c[5:])
            if n is None or n not in P.SYNC_MODE_VALUES:
                return []               # молчание, как прошивка (парсер вернул 0)
            self.sync_mode = n
            return [f"ok:sync={n}"]
        if c.startswith("hold="):
            n = _arg_int(c[5:])
            if n is None or n not in (0, 1):
                return []
            # Как CMD_SET_HOLD: цель, точка скана и уже выданный событийный
            # кадр сохраняются — меняется только питание обмоток. PID и рампа
            # с нуля: за паузу без тока накопленное состояние протухло.
            self.hold = n
            if n:
                # Ток вернулся — прошивка просит доводочный шаг, чтобы вал сел
                # в цель с той же точностью, что и при штатном приходе.
                self.was_outside_db = 1
            self._control_reset()
            return [f"ok:hold={n}"]
        if c == "sync":
            # in= и n= у модели всегда нули: физического входа нет. out= —
            # уровень SYNC_OUT, который прошивка поднимает в точке скана.
            return [f"sync={self.sync_mode} in=0 out={self.sync_out} n={self.sync_edges}"]
        if c == "om":
            # Запрос состояния, а не подтверждение команды, — поэтому без
            # префикса ok:, как у sync (main.c CMD_GET_OUTPUT_MODE:
            # SendResponse("om=%u\r\n", g_telem_mode)).
            return [f"om={self.telem_mode}"]
        if c == "hold":
            # main.c CMD_GET_HOLD: SendResponse("hold=%u\r\n", g_hold).
            return [f"hold={self.hold}"]
        if c == "mcfg":
            # ready у модели всегда 1: живого TMC2209 нет, а значит нет и
            # ответов err:not ready на irun/ihold/icur/mstep.
            return [f"mode=STEP_DIR run={self.irun} hold={self.ihold} "
                    f"microsteps={self.microsteps} ready={1 if self.ready else 0}"]
        if c == "diag":
            # Как прошивка: на движущемся вале — err:busy, иначе ok:diag и
            # отчёт. На плате отчёт приходит отдельной строкой через ~30 мс.
            if self.is_busy():
                return ["err:busy stop motor first"]
            return ["ok:diag", f"enc:ok n=16/16 spread=0.000 pos={self.cur_deg:.2f}"]
        if c.startswith("irun "):
            r = _parse_int(c, 5)
            if r is None or not P.current_ok(r[0]):
                return []
            self.irun = r[0]
            return [f"ok:irun={self.irun}"]
        if c.startswith("ihold "):
            r = _parse_int(c, 6)
            if r is None or not P.current_ok(r[0]):
                return []
            self.ihold = r[0]
            return [f"ok:ihold={self.ihold}"]
        if c.startswith("icur "):
            r = _parse_int(c, 5)
            if r is None or not P.current_ok(r[0]):
                return []
            run, i = r
            r = _parse_int(c, _skip_spaces(c, i))
            if r is None or not P.current_ok(r[0]):
                return []
            self.irun, self.ihold = run, r[0]
            return [f"ok:icur={self.irun},{self.ihold}"]
        if c.startswith("mstep "):
            r = _parse_int(c, 6)
            if r is None or r[0] < 0 or r[0] > P.MSTEP_ARG_MAX:
                # Парсер прошивки проверяет обе границы ДО приведения к
                # uint16 (cmd_parser.c: `v < 0 || v > 65535`), возвращает 0 —
                # и команда до обработчика не доходит вовсе: ни ok:, ни
                # err:busy, ни err:bad arg, просто молчание.
                return []
            # Порядок как в CMD_SET_MSTEP: сначала «мотор крутится», и только
            # потом проверка самого значения драйвером.
            if self.is_moving():
                return ["err:busy stop motor first"]
            if not P.mstep_ok(r[0]):
                return ["err:bad arg (1/2/4/8/16/32/64/128/256)"]
            self.microsteps = r[0]
            return [f"ok:mstep={self.microsteps}"]

        return ["err:unknown"]

    def _cmd_scan(self, arg: str) -> list[str]:
        """scan=start,end|+|-,step,delay — разбор строго как в Cmd_Parse().

        Структура (запятые на местах, числа читаемы) — условие ответа вообще:
        не сошлась, значит парсер вернул 0 и прошивка молчит. Уже разобранные
        значения проверяются на смысл и дают явный err:scan.
        """
        r = _parse_float(arg, 0)
        if r is None:
            return []
        start, i = r
        if i >= len(arg) or arg[i] != ",":
            return []
        i += 1

        inf = 0
        end = 0.0
        if i + 1 < len(arg) and arg[i] in "+-" and arg[i + 1] == ",":
            inf = 1 if arg[i] == "+" else -1
            i += 2
        else:
            r = _parse_float(arg, i)
            if r is None:
                return []
            end, i = r
            if i >= len(arg) or arg[i] != ",":
                return []
            i += 1

        r = _parse_float(arg, i)
        if r is None:
            return []
        step, i = r
        if i >= len(arg) or arg[i] != ",":
            return []
        i += 1
        r = _parse_int(arg, i)
        if r is None:
            return []
        delay = r[0]
        if delay < 0 or delay > 65535:
            return []                   # вне uint16 — парсер вернул 0, молчание

        # Валидация → err:scan (прошивка отвечает явной ошибкой). Протяжённость
        # сектора отсчитывается от start в сторону возрастания угла по кольцу,
        # поэтому end < start — это сектор через ноль, а не ошибка. Отвергается
        # только вырожденный сектор (start и end — одна точка кольца, в т.ч.
        # scan=0,360): полный круг задаётся бесконечным сканом.
        if step <= 0 or delay == 0:
            return ["err:scan"]
        span = 0.0
        if not inf:
            span = _wrap360(end - start)
            if span <= 0.0:
                return ["err:scan"]

        start = _wrap360(start)
        self.cont_dir = 0
        self.scan_st = SCAN_MOVING
        self.scan_inf = inf
        self.scan_start = start
        self.scan_span = span
        self.scan_u = 0.0
        self.scan_step = step
        self.scan_delay_ms = delay
        self.scan_sp = start
        self.target_deg = start
        self.scan_dwell_ms = 0.0
        self.scan_dir = inf if inf else 1
        self._pid_reset()               # как прошивка: PID_Reset перед стартом
        self._target_reset_reached()
        self.syncin_trig = 0            # фронт до старта скана не считается
        self.sync_out = 0
        # Эхо — приведённые к [0,360) значения, как у прошивки
        if inf:
            return [f"ok:scan={start:.2f},{'+' if inf > 0 else '-'},{step:.2f},{delay}"]
        return [f"ok:scan={start:.2f},{_wrap360(start + span):.2f},{step:.2f},{delay}"]

    # ── Шаг модели ─────────────────────────────────────────────────────────
    def _limit_velocity(self, v_des: float, err: float | None, dt_s: float) -> float:
        """Motion_Limit(): ограничение скорости (vmax) и ускорения (accel).

        v_des/возврат — °/с (прошивка считает то же самое в градусах за такт).
        v_des приходит с выхода ПИД в режиме слежения и равен ±vmax в джоге.
        err (°) задаёт тормозной конверт sqrt(2·a·|err|), None — конверт не
        применять (непрерывное вращение, have_err = 0).
        """
        v_des = max(-self.vmax, min(self.vmax, v_des))
        if self.accel > 0:
            if err is not None:
                vbrake = math.sqrt(2.0 * self.accel * abs(err))
                v_des = max(-vbrake, min(vbrake, v_des))
            dv = self.accel * dt_s
            self.vel += max(-dv, min(dv, v_des - self.vel))
        else:
            self.vel = v_des
        return self.vel

    def tick(self, dt_ms: float) -> None:
        """Один шаг модели. Порядок как в главном цикле main.c: сначала
        управление мотором, затем скан (он опирается на «приехали» этого же
        тика). Телеметрия — отдельным вызовом telemetry_tick()."""
        if dt_ms <= 0:
            return
        self._motor_tick(dt_ms)
        self._scan_tick(dt_ms)

    def _motor_tick(self, dt_ms: float) -> None:
        """MotorControl_Tick(): движение к цели, мёртвая зона, приход в точку."""
        dt_s = dt_ms / 1000.0
        self.last_u = 0.0
        # Доводочная серия прошлого тика к этому моменту отработана: прошивка
        # растягивает её ровно на один период опроса (tmc2209_motor.c:283).
        self.snap_busy = 0

        if not self.enabled:
            # Разрешения нет: мотор стоит, ожидание «приехали» сброшено —
            # как в прошивке, где Target_ResetReached() зовётся каждый тик.
            self._target_reset_reached()
            return
        if not self.hold:
            # Удержание снято: контур на паузе, цель и состояние «приехали»
            # не трогаем (после hold=1 второго кадра по той же точке не будет).
            return

        if self.cont_dir != 0:
            # Непрерывное вращение: понятия «цель достигнута» нет.
            self._target_reset_reached()
            v = self._limit_velocity(self.cont_dir * self.vmax, None, dt_s)
            delta = v * dt_s
            # Кольцевая координата: угол заворачивается на нуле, как в прошивке
            # (g_ol_pos/g_target_deg через wrap360). Цель едет вместе с валом,
            # поэтому ошибка позиции в телеметрии остаётся прежней.
            self.cur_deg = _wrap360(self.cur_deg + delta)
            self.target_deg = _wrap360(self.target_deg + delta)
            self.last_u = delta
            return

        # Слежение за целью. Ошибка — по кратчайшему пути, поэтому у цели 10°
        # и позиции 350° модель идёт вперёд через ноль на 20°, а не назад на
        # 340°. Желаемую скорость задаёт ПИД, дальше её режет профиль v=/a=
        # (main.c:976-978: pid = PID_Update(...); v = Motion_Limit(pid, err, 1)).
        err = self._err()
        if abs(err) > DEADBAND_DEG:
            self.was_outside_db = 1    # g_was_outside_db: ждём доводочный шаг
            self._at_target = 0        # Target_LeftDeadband()
            pid = self._pid_update(err, dt_s)       # градусы за такт контура
            v = self._limit_velocity(pid / CONTROL_DT_S, err, dt_s)
            # Ограничения «не перелететь цель» у прошивки нет: ApplyVelocity
            # честно крутит вал на скомандованной скорости, а перелёт (если
            # регулятор его допустил) отрабатывается следующими тиками.
            delta = v * dt_s
            self.cur_deg = _wrap360(self.cur_deg + delta)
            self.last_u = delta
            return

        # В мёртвой зоне. Прошивка, войдя в неё, сначала делает доводочный шаг
        # (snap: DegToSteps(err) микрошагов) и только следующим тиком, когда
        # шаги отработаны, засчитывает приход в цель — иначе кадр ушёл бы с
        # позицией, которую вал ещё не занял.
        final_move = 0
        if self.was_outside_db:
            if abs(err) > SNAP_MIN_DEG:
                # Квантования по микрошагу у модели нет: считаем, что доводка
                # сажает вал ровно в цель (на плате точность — один микрошаг).
                self.cur_deg = _wrap360(self.cur_deg + err)
                self.last_u = err
                final_move = 1
                # Импульсы серии ещё идут — до следующего тика вал «в движении»
                self.snap_busy = 1
            self.was_outside_db = 0
        if self.scan_st == SCAN_MOVING:
            self.sync_out = 1          # SYNC_OUT HIGH: точка скана достигнута
            self.scan_sp = self.target_deg
            self.scan_st = SCAN_DELAY
            self.scan_dwell_ms = 0.0
            self.syncin_trig = 0
        if not final_move:
            self._target_mark_reached()
        self._control_reset()          # PID и рампа в цели — с нуля

    def _scan_tick(self, dt_ms: float) -> None:
        """Scan_Tick(): переход к следующей точке по таймеру и/или SYNC_IN."""
        if self.scan_st != SCAN_DELAY:
            return
        # Удержание снято — скан заморожен: таймер delay не тикает, точка не
        # меняется (иначе плата «проехала» бы точку с обесточенным мотором).
        if not self.hold:
            return

        timer_ok = False
        if self.scan_delay_ms != 0:
            self.scan_dwell_ms += dt_ms
            timer_ok = self.scan_dwell_ms >= self.scan_delay_ms
        # sync=1: переход только по фронту SYNC_IN, а пина в модели нет —
        # скан честно стоит на точке (в sync=2 delay работает как тайм-аут).
        if self.sync_mode == SYNC_ADV_TIMER:
            go = timer_ok
        elif self.sync_mode == SYNC_ADV_EXT:
            go = bool(self.syncin_trig)
        elif self.sync_mode == SYNC_ADV_EXT_TIMEOUT:
            go = bool(self.syncin_trig) or timer_ok
        else:
            go = False              # как default в switch прошивки
        if not go:
            return
        self.syncin_trig = 0

        if self.scan_inf:
            # Бесконечный скан: угол просто заворачивается на нуле, границ нет
            self.scan_sp = _wrap360(self.scan_sp + self.scan_inf * self.scan_step)
        else:
            # Зигзаг в координате сектора u ∈ [0, span] — разворот на краях
            # не зависит от того, пересекает ли сектор ноль (как Scan_Tick).
            u = self.scan_u + self.scan_dir * self.scan_step
            if self.scan_dir > 0 and u > self.scan_span:
                u = (self.scan_span - self.scan_step
                     if self.scan_u >= self.scan_span else self.scan_span)
                self.scan_dir = -1
            elif self.scan_dir < 0 and u < 0.0:
                u = self.scan_step if self.scan_u <= 0.0 else 0.0
                self.scan_dir = 1
            self.scan_u = u
            self.scan_sp = _wrap360(self.scan_start + u)
        self.target_deg = self.scan_sp
        self.scan_st = SCAN_MOVING
        self.scan_dwell_ms = 0.0
        # Профиль движения не сбрасываем: между точками рампа скорости
        # продолжается плавно, а PID стартует с чистым интегралом (Scan_Tick)
        self._pid_reset()
        # Каждая точка скана — отдельная цель: один событийный кадр на точку
        self._target_reset_reached()
        self.sync_out = 0

    # ── Телеметрия ─────────────────────────────────────────────────────────
    def telemetry_tick(self, dt_ms: float) -> str | None:
        """Telemetry_Tick(): строка кадра на этот тик либо None.

        Периодический источник считает время до op= (при debug=1 период не
        меньше OUTPUT_PERIOD_MS_DEBUG_MIN), событийный отдаёт взведённый кадр.
        Совпали в одном тике — уходит один кадр, событийный: у прошивки ev
        перекрывает периодический и подменяет позицию снимком момента прихода.
        """
        evt = bool(self._evt_pending)
        periodic = False
        if self.telem_mode != TELEM_SRC_TARGET and self.op_ms != 0:
            period = (max(self.op_ms, OUTPUT_PERIOD_MS_DEBUG_MIN)
                      if self.debug else self.op_ms)
            self._telem_cnt += dt_ms
            if self._telem_cnt >= period:
                self._telem_cnt = 0.0
                periodic = True
        else:
            self._telem_cnt = 0.0
        if not periodic and not evt:
            return None
        self._evt_pending = 0
        return self.telemetry_line(event=evt)

    def telemetry_line(self, event: bool = False) -> str:
        """Кадр телеметрии. event=True — кадр по приходу в цель (метка ev:1).

        В событийном кадре позиция и цель берутся снимком момента прихода:
        на плате кадр может уйти на 1–2 тика позже, а хосту нужен угол именно
        того момента, к которому привязан замер.
        """
        cp = self._evt_deg if event else self.cur_deg
        tp = self._evt_tp if event else self.target_deg
        pe = _wrap180(tp - cp)    # как прошивка: ошибка по кратчайшему пути
        line = P.format_telemetry(
            cp=cp, tp=tp, pe=pe, u=self.last_u,
            mode=self.mode, ec=self.ec, kp=self.kp, ki=self.ki, kd=self.kd,
            drp=self.drp, debug=bool(self.debug),
            vmax=self.vmax, accel=self.accel,
        )
        # Поле ev:1 прошивка дописывает в конец обеих форм строки (main.c)
        return (line + ",ev:1") if event else line


def _wrap180(x: float) -> float:
    """Ошибка по кратчайшему пути, (-180, 180] — как shortest_path_err()."""
    return P.wrap180(x)


def _wrap360(x: float) -> float:
    """Приводит угол к кольцевому диапазону [0, 360) — как wrap360() прошивки."""
    return P.wrap360(x)
