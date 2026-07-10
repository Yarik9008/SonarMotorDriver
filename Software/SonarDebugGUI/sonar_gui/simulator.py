"""FirmwareSimulator — Python-модель прошивки FW_SonarMotorDriver.

Чистая логика без Qt: разбирает те же команды и отдаёт побитово те же строки
ответов/телеметрии, что и железо (иначе разбор в GUI не проверился бы).
Обёртку с таймерами даёт transport/sim_transport.py.

Поведение воспроизводит прошивку:
- позиция cur_deg плавно идёт к target_deg с ограничением скорости (v=)
  и ускорения (a=, 0 = мгновенно) — как профиль движения прошивки;
- непрерывное вращение t=+/t=-;
- сектор (зигзаг) и бесконечный скан с задержкой delay в каждой точке;
- «молчание» (пустой список) на аргумент вне диапазона, как у прошивки.
"""
from __future__ import annotations

import math

from . import protocol as P

DEADBAND_DEG = 0.05


class FirmwareSimulator:
    def __init__(self) -> None:
        d = P.DEFAULTS
        self.enabled = True
        self.ready = True
        self.mode = "cl"            # sim всегда замкнутый контур
        self.ec = 0
        self.drp = 0

        self.cur_deg = d.target_deg
        self.target_deg = d.target_deg
        self.last_u = 0.0

        self.cont_dir = 0           # 0 / +1 / -1 — непрерывное вращение

        # Скан
        self.scan_active = False
        self.scan_start = 0.0
        self.scan_end = 0.0
        self.scan_step = 0.0
        self.scan_delay_ms = 0
        self.scan_inf = False
        self.scan_dir = 1
        self.scan_sp = 0.0
        self.scan_dwell_ms = 0.0

        self.kp = d.kp
        self.ki = d.ki
        self.kd = d.kd
        self.op_ms = d.op_ms
        self.debug = d.debug
        self.irun = d.irun
        self.ihold = d.ihold
        self.microsteps = d.microsteps

        # Профиль движения (v= / a=), как в прошивке
        self.vmax = d.vmax          # предел скорости, °/с
        self.accel = d.accel        # предел ускорения, °/с² (0 = мгновенно)
        self.vel = 0.0              # текущая скорость модели, °/с (знаковая)

    # ── Стартовые сообщения ────────────────────────────────────────────────
    def boot_lines(self) -> list[str]:
        """Строки, которые прошивка шлёт при старте (диагностика энкодера).

        Реальная прошивка перед разрешением движения делает серию чтений
        BiSS-C и отправляет enc:ok / err:enc. У модели диагностика всегда
        успешна — формат строки повторяет прошивку байт-в-байт.
        """
        return [f"enc:ok n=16/16 spread=0.000 pos={self.cur_deg:.2f}"]

    # ── Состояние ──────────────────────────────────────────────────────────
    def is_moving(self) -> bool:
        if self.cont_dir != 0 or self.scan_active:
            return True
        return abs(self.target_deg - self.cur_deg) > DEADBAND_DEG

    def scan_range(self):
        """Текущий сектор скана для подсветки на диаграмме, либо None."""
        if not self.scan_active or self.scan_inf:
            return None
        return (self.scan_start, self.scan_end)

    # ── Приём команды ──────────────────────────────────────────────────────
    def handle_command(self, line: str) -> list[str]:
        """Возвращает строки ответа. Пустой список = «молчание» прошивки."""
        c = line.strip()
        if not c:
            return []

        if c == "en":
            self.enabled = True
            self.cont_dir = 0
            return ["ok:en"]
        if c == "dis":
            self.enabled = False
            self.cont_dir = 0
            self.scan_active = False
            return ["ok:dis"]
        if c == "stop":
            self.cont_dir = 0
            self.scan_active = False
            self.target_deg = self.cur_deg
            self.vel = 0.0
            return ["ok:stop"]
        if c == "t=+":
            self.cont_dir = 1
            self.scan_active = False
            return ["ok:t=+"]
        if c == "t=-":
            self.cont_dir = -1
            self.scan_active = False
            return ["ok:t=-"]
        if c == "mcfg":
            return [f"mode=STEP_DIR run={self.irun} hold={self.ihold} "
                    f"microsteps={self.microsteps} ready={1 if self.ready else 0}"]
        if c == "diag":
            # Как прошивка: на движущемся вале — err:busy, иначе ok:diag + отчёт
            if self.is_moving():
                return ["err:busy stop motor first"]
            return ["ok:diag", f"enc:ok n=16/16 spread=0.000 pos={self.cur_deg:.2f}"]

        if c.startswith("t="):
            v = _f(c[2:])
            if v is None:
                return []
            self.target_deg = v
            self.cont_dir = 0
            self.scan_active = False
            return [f"ok:t={v:.2f}"]
        if c.startswith("kp="):
            v = _f(c[3:])
            if v is None:
                return []
            self.kp = v
            return [f"ok:kp={v:.4f}"]
        if c.startswith("ki="):
            v = _f(c[3:])
            if v is None:
                return []
            self.ki = v
            return [f"ok:ki={v:.4f}"]
        if c.startswith("kd="):
            v = _f(c[3:])
            if v is None:
                return []
            self.kd = v
            return [f"ok:kd={v:.4f}"]
        if c.startswith("v="):
            v = _f(c[2:])
            if v is None:
                return []
            if not P.speed_ok(v):
                return [f"err:bad arg (v={P.SPEED_MIN_DEG_S:g}..{P.MAX_SPEED_DEG_S:g})"]
            self.vmax = v
            return [f"ok:v={v:.1f}"]
        if c.startswith("a="):
            v = _f(c[2:])
            if v is None:
                return []
            if not P.accel_ok(v):
                return [f"err:bad arg (a=0..{P.ACCEL_MAX_DEG_S2:g})"]
            self.accel = v
            return [f"ok:a={v:.1f}"]
        if c.startswith("op="):
            n = _i(c[3:])
            if n is None or not P.op_ok(n):
                return []
            self.op_ms = n
            return [f"ok:op={n}"]
        if c.startswith("debug="):
            if c[6:] not in ("0", "1"):
                return []
            self.debug = int(c[6:])
            return [f"ok:debug={self.debug}"]
        if c.startswith("scan="):
            return self._cmd_scan(c[5:])
        if c.startswith("irun "):
            n = _i(c[5:])
            if n is None or not P.current_ok(n):
                return []
            self.irun = n
            return [f"ok:irun={n}"]
        if c.startswith("ihold "):
            n = _i(c[6:])
            if n is None or not P.current_ok(n):
                return []
            self.ihold = n
            return [f"ok:ihold={n}"]
        if c.startswith("icur "):
            parts = c[5:].split()
            if len(parts) != 2:
                return []
            run, hold = _i(parts[0]), _i(parts[1])
            if run is None or hold is None:
                return []
            if not P.current_ok(run) or not P.current_ok(hold):
                return []
            self.irun, self.ihold = run, hold
            return [f"ok:icur={run},{hold}"]
        if c.startswith("mstep "):
            if self.is_moving():
                return ["err:busy stop motor first"]
            n = _i(c[6:])
            if not P.mstep_ok(n):
                return ["err:bad arg (1/2/4/8/16/32/64/128/256)"]
            self.microsteps = n
            return [f"ok:mstep={n}"]

        return ["err:unknown"]

    def _cmd_scan(self, arg: str) -> list[str]:
        parts = arg.split(",")
        if len(parts) != 4:
            return []                       # структуру не разобрать → молчание
        start = _f(parts[0])
        step = _f(parts[2])
        delay = _i(parts[3])
        if start is None or step is None or delay is None:
            return []
        inf = parts[1] in ("+", "-")
        end = None
        if not inf:
            end = _f(parts[1])
            if end is None:
                return []
        # Валидация → err:scan (прошивка отвечает явной ошибкой)
        if step <= 0 or delay <= 0 or (not inf and not (start < end)):
            return ["err:scan"]

        # Кратчайший заход в сектор: коллапсируем накопленные обороты к кадру
        # старта. После долгого непрерывного вращения вал иначе «разматывал» бы
        # многие обороты до первой точки (как homing-коллапс прошивки, main.c
        # строки 806–808) — визуально скан выглядел бы зависшим.
        self.cur_deg = start + _wrap180(self.cur_deg - start)

        self.scan_active = True
        self.scan_inf = inf
        self.scan_start = start
        self.scan_step = step
        self.scan_delay_ms = delay
        self.cont_dir = 0
        self.scan_sp = start
        self.target_deg = start
        self.scan_dwell_ms = 0.0
        if inf:
            self.scan_dir = 1 if parts[1] == "+" else -1
            self.scan_end = start
            return [f"ok:scan={start:.2f},{parts[1]},{step:.2f},{delay}"]
        self.scan_dir = 1
        self.scan_end = end
        return [f"ok:scan={start:.2f},{end:.2f},{step:.2f},{delay}"]

    # ── Шаг модели ─────────────────────────────────────────────────────────
    def _limit_velocity(self, v_des: float, err: float | None, dt_s: float) -> float:
        """Ограничение скорости (vmax) и ускорения (accel) — как в прошивке.

        v_des/возврат — °/с. err (°) задаёт тормозной конверт sqrt(2·a·|err|),
        None — конверт не применять (непрерывное вращение).
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
        if not self.enabled or dt_ms <= 0:
            self.last_u = 0.0
            return

        dt_s = dt_ms / 1000.0

        if self.cont_dir != 0:
            v = self._limit_velocity(self.cont_dir * self.vmax, None, dt_s)
            delta = v * dt_s
            # Угол НЕ оборачивается на 360°: как в прошивке (g_target_deg += v,
            # main.c ~764), позиция накапливается многооборотно — cp растёт через
            # 360°/720°/… и телеметрия с графиком отражают суммарный поворот.
            # Размотку перед сканом снимает коллапс к кратчайшему пути в _cmd_scan,
            # поэтому накопление здесь безопасно.
            self.cur_deg += delta
            # Предохранитель от неограниченного роста (как прошивка при |pos|>1e7).
            if self.cur_deg > 1e7 or self.cur_deg < -1e7:
                self.cur_deg = 0.0
            self.target_deg = self.cur_deg
            self.last_u = delta
            return

        if self.scan_active:
            self._scan_tick(dt_ms)

        # Слежение за целью с ограничением скорости и ускорения
        err = self.target_deg - self.cur_deg
        v = self._limit_velocity(err / dt_s, err, dt_s)
        delta = v * dt_s
        if abs(delta) >= abs(err):
            delta = err            # не перелетаем цель за один тик модели
            self.vel = 0.0 if self.accel > 0 else self.vel
        self.cur_deg += delta
        self.last_u = delta

    def _scan_tick(self, dt_ms: float) -> None:
        # Ждём прихода в текущую точку, затем выдерживаем задержку и идём дальше.
        if abs(self.target_deg - self.cur_deg) > DEADBAND_DEG:
            return
        self.scan_dwell_ms += dt_ms
        if self.scan_dwell_ms < self.scan_delay_ms:
            return
        self.scan_dwell_ms = 0.0
        if self.scan_inf:
            self.scan_sp += self.scan_dir * self.scan_step
        else:
            if self.scan_dir > 0:
                self.scan_sp += self.scan_step
                if self.scan_sp >= self.scan_end:
                    self.scan_sp = self.scan_end
                    self.scan_dir = -1
            else:
                self.scan_sp -= self.scan_step
                if self.scan_sp <= self.scan_start:
                    self.scan_sp = self.scan_start
                    self.scan_dir = 1
        self.target_deg = self.scan_sp

    # ── Телеметрия ─────────────────────────────────────────────────────────
    def telemetry_line(self) -> str:
        pe = self.target_deg - self.cur_deg
        return P.format_telemetry(
            cp=self.cur_deg, tp=self.target_deg, pe=pe, u=self.last_u,
            mode=self.mode, ec=self.ec, kp=self.kp, ki=self.ki, kd=self.kd,
            drp=self.drp, debug=bool(self.debug),
            vmax=self.vmax, accel=self.accel,
        )


def _wrap180(x: float) -> float:
    """Приводит угол к диапазону (-180, 180] — для кратчайшего пути."""
    return (x + 180.0) % 360.0 - 180.0


def _f(s: str):
    try:
        return float(s)
    except ValueError:
        return None


def _i(s: str):
    try:
        return int(s)
    except ValueError:
        return None
