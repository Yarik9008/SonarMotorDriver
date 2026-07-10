"""MotorPanel — вкл/выкл, стоп, целевой угол, скорость/ускорение, джог.

Джог: обычный (текущая скорость v=), быстрый и медленный (пресеты
P.JOG_FAST_DEG_S / P.JOG_SLOW_DEG_S). Кнопки работают по удержанию:
нажатие — движение, отпускание — стоп (и восстановление настроенной скорости).
"""
from __future__ import annotations

from typing import Callable

from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QDoubleSpinBox, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QPushButton, QSizePolicy,
                               QVBoxLayout)

from .. import protocol as P


def _mono(px: int) -> QFont:
    f = QFont()
    f.setFamilies(["Cascadia Mono", "Consolas", "monospace"])
    f.setPixelSize(px)
    return f


class MotorPanel(QGroupBox):
    def __init__(self, send: Callable[[str], bool], parent=None):
        super().__init__("МОТОР", parent)
        self._send = send
        d = P.DEFAULTS
        root = QVBoxLayout(self)
        root.setSpacing(6)

        # Вкл / Выкл — равные по ширине
        row1 = QHBoxLayout()
        b_en = QPushButton("Включить")
        b_en.clicked.connect(lambda: self._send(P.cmd_enable()))
        b_dis = QPushButton("Выключить")
        b_dis.clicked.connect(lambda: self._send(P.cmd_disable()))
        row1.addWidget(b_en, 1)
        row1.addWidget(b_dis, 1)
        root.addLayout(row1)

        # Аварийный стоп — во всю ширину панели
        b_stop = QPushButton("СТОП")
        b_stop.setObjectName("stopButton")
        b_stop.clicked.connect(lambda: self._send(P.cmd_stop()))
        root.addWidget(b_stop)

        # Целевой угол
        row_t = QHBoxLayout()
        lbl_t = QLabel("Цель, °")
        lbl_t.setProperty("dim", "true")
        self._target = QDoubleSpinBox()
        self._target.setRange(-1_000_000.0, 1_000_000.0)
        self._target.setDecimals(2)
        self._target.setValue(0.0)
        self._target.setSizePolicy(QSizePolicy.Policy.Expanding,
                                   QSizePolicy.Policy.Fixed)
        # Явный минимум: иначе min-подсказка спинбокса с диапазоном ±1e6
        # распирает панель шире 400px и левая колонка обрезается (§0)
        self._target.setMinimumWidth(90)
        b_go = QPushButton("→ Перейти")
        b_go.clicked.connect(self._go)
        row_t.addWidget(lbl_t)
        row_t.addWidget(self._target, 1)
        row_t.addWidget(b_go)
        root.addLayout(row_t)

        # Профиль движения: скорость (v=) и ускорение (a=) с эхо из телеметрии
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        grid.setColumnStretch(1, 1)

        self._echo: dict[str, QLabel] = {}

        lbl_v = QLabel("Скорость, °/с")
        lbl_v.setProperty("dim", "true")
        grid.addWidget(lbl_v, 0, 0)
        self._speed = QDoubleSpinBox()
        self._speed.setRange(P.SPEED_MIN_DEG_S, P.MAX_SPEED_DEG_S)
        self._speed.setDecimals(1)
        self._speed.setValue(d.vmax)
        self._speed.setToolTip("Предел скорости движения (команда v=)")
        grid.addWidget(self._speed, 0, 1)
        b_v = QPushButton("✓")
        b_v.setFixedWidth(34)
        b_v.setToolTip("Применить скорость")
        b_v.clicked.connect(lambda: self._send(P.cmd_speed(self._speed.value())))
        grid.addWidget(b_v, 0, 2)
        echo_v = QLabel("—")
        echo_v.setProperty("dim", "true")
        echo_v.setFont(_mono(12))
        echo_v.setMinimumWidth(72)
        grid.addWidget(echo_v, 0, 3)
        self._echo["v"] = echo_v

        lbl_a = QLabel("Ускорение, °/с²")
        lbl_a.setProperty("dim", "true")
        grid.addWidget(lbl_a, 1, 0)
        self._accel = QDoubleSpinBox()
        self._accel.setRange(0.0, P.ACCEL_MAX_DEG_S2)
        self._accel.setDecimals(1)
        self._accel.setValue(d.accel)
        self._accel.setToolTip("Предел ускорения (команда a=), 0 = без рампы")
        grid.addWidget(self._accel, 1, 1)
        b_a = QPushButton("✓")
        b_a.setFixedWidth(34)
        b_a.setToolTip("Применить ускорение")
        b_a.clicked.connect(lambda: self._send(P.cmd_accel(self._accel.value())))
        grid.addWidget(b_a, 1, 2)
        echo_a = QLabel("—")
        echo_a.setProperty("dim", "true")
        echo_a.setFont(_mono(12))
        echo_a.setMinimumWidth(72)
        grid.addWidget(echo_a, 1, 3)
        self._echo["a"] = echo_a

        root.addLayout(grid)

        # Толчковое вращение: удержание = движение, отпускание = стоп.
        # Обычный джог — на текущей настроенной скорости.
        row_jog = QHBoxLayout()
        b_ccw = QPushButton("⟲ Против (t=-)")
        b_cw = QPushButton("⟳ По час. (t=+)")
        b_ccw.pressed.connect(lambda: self._send(P.cmd_jog("-")))
        b_ccw.released.connect(lambda: self._send(P.cmd_stop()))
        b_cw.pressed.connect(lambda: self._send(P.cmd_jog("+")))
        b_cw.released.connect(lambda: self._send(P.cmd_stop()))
        tip = "Удерживайте — непрерывное вращение, отпустите — стоп"
        b_ccw.setToolTip(tip)
        b_cw.setToolTip(tip)
        # Позволяем джогам сжиматься — ширина панели ограничена 400px (§0)
        b_ccw.setMinimumWidth(80)
        b_cw.setMinimumWidth(80)
        row_jog.addWidget(b_ccw, 1)
        row_jog.addWidget(b_cw, 1)
        root.addLayout(row_jog)

        # Быстрый / медленный джог — пресеты скорости
        row_fast = QHBoxLayout()
        b_fast_ccw = QPushButton("⟲⟲ Быстро")
        b_fast_cw = QPushButton("Быстро ⟳⟳")
        self._bind_preset_jog(b_fast_ccw, "-", P.JOG_FAST_DEG_S)
        self._bind_preset_jog(b_fast_cw, "+", P.JOG_FAST_DEG_S)
        row_fast.addWidget(b_fast_ccw, 1)
        row_fast.addWidget(b_fast_cw, 1)
        root.addLayout(row_fast)

        row_slow = QHBoxLayout()
        b_slow_ccw = QPushButton("⟲ Медленно")
        b_slow_cw = QPushButton("Медленно ⟳")
        self._bind_preset_jog(b_slow_ccw, "-", P.JOG_SLOW_DEG_S)
        self._bind_preset_jog(b_slow_cw, "+", P.JOG_SLOW_DEG_S)
        row_slow.addWidget(b_slow_ccw, 1)
        row_slow.addWidget(b_slow_cw, 1)
        root.addLayout(row_slow)

    # ── Джог с пресетом скорости ────────────────────────────────────────────
    def _bind_preset_jog(self, btn: QPushButton, sign: str, speed: float) -> None:
        btn.setToolTip(f"Удерживайте — вращение {speed:g} °/с, отпустите — стоп")
        btn.setMinimumWidth(80)
        btn.pressed.connect(lambda: self._preset_jog_start(sign, speed))
        btn.released.connect(self._preset_jog_stop)

    def _preset_jog_start(self, sign: str, speed: float) -> None:
        self._send(P.cmd_speed(speed))
        self._send(P.cmd_jog(sign))

    def _preset_jog_stop(self) -> None:
        self._send(P.cmd_stop())
        # Возвращаем настроенную пользователем скорость
        self._send(P.cmd_speed(self._speed.value()))

    # ── Обновления ──────────────────────────────────────────────────────────
    def update_from_telemetry(self, data: dict) -> None:
        if "v" in data:
            self._echo["v"].setText(f"= {data['v']:.1f}")
        if "a" in data:
            self._echo["a"].setText(f"= {data['a']:.1f}")

    def _go(self) -> None:
        self._send(P.cmd_target(self._target.value()))
