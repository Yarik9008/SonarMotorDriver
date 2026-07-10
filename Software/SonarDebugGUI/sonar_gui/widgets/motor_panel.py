"""MotorPanel — вкл/выкл, целевой угол, скорость/ускорение, джог."""
from __future__ import annotations

from typing import Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import (QButtonGroup, QDoubleSpinBox, QGroupBox,
                               QHBoxLayout, QLabel, QPushButton, QRadioButton,
                               QSizePolicy, QVBoxLayout)

from .. import protocol as P
from ..device_state import DeviceState
from ..metrics import METRICS, label
from ..theme import COLORS, mono_font
from .param_row import ParamRow


class MotorPanel(QGroupBox):
    def __init__(self, send: Callable[[str], bool], parent=None):
        super().__init__("МОТОР", parent)
        self._send = send
        d = P.DEFAULTS
        root = QVBoxLayout(self)
        root.setSpacing(6)

        row1 = QHBoxLayout()
        b_en = QPushButton("Включить")
        b_en.clicked.connect(lambda: self._send(P.cmd_enable()))
        b_dis = QPushButton("Выключить")
        b_dis.clicked.connect(lambda: self._send(P.cmd_disable()))
        row1.addWidget(b_en, 1)
        row1.addWidget(b_dis, 1)
        root.addLayout(row1)

        row_t = QHBoxLayout()
        lbl_t = QLabel(label("tp"))
        lbl_t.setProperty("dim", "true")
        lbl_t.setToolTip(METRICS["tp"].tooltip)
        self._target = QDoubleSpinBox()
        self._target.setRange(-1_000_000.0, 1_000_000.0)
        self._target.setDecimals(2)
        self._target.setValue(0.0)
        self._target.setSizePolicy(QSizePolicy.Policy.Expanding,
                                   QSizePolicy.Policy.Fixed)
        self._target.setMinimumWidth(90)
        self._target.installEventFilter(self)
        b_go = QPushButton("→ Перейти")
        b_go.clicked.connect(self._go)
        row_t.addWidget(lbl_t)
        row_t.addWidget(self._target, 1)
        row_t.addWidget(b_go)
        self._echo_tp = QLabel("—")
        self._echo_tp.setProperty("dim", "true")
        self._echo_tp.setFont(mono_font(12))
        self._echo_tp.setMinimumWidth(72)
        row_t.addWidget(self._echo_tp)
        root.addLayout(row_t)

        self._speed = QDoubleSpinBox()
        self._speed.setRange(P.SPEED_MIN_DEG_S, P.MAX_SPEED_DEG_S)
        self._speed.setDecimals(1)
        self._speed.setValue(d.vmax)
        self._row_v = ParamRow(
            label("v"), self._speed,
            lambda: self._send(P.cmd_speed(self._speed.value())),
            METRICS["v"].tooltip)
        self._row_v.set_format("{:.1f}")
        root.addWidget(self._row_v)

        self._accel = QDoubleSpinBox()
        self._accel.setRange(0.0, P.ACCEL_MAX_DEG_S2)
        self._accel.setDecimals(1)
        self._accel.setValue(d.accel)
        self._row_a = ParamRow(
            label("a"), self._accel,
            lambda: self._send(P.cmd_accel(self._accel.value())),
            METRICS["a"].tooltip)
        self._row_a.set_format("{:.1f}")
        root.addWidget(self._row_a)

        row_speed = QHBoxLayout()
        lbl_j = QLabel("Скорость джога")
        lbl_j.setProperty("dim", "true")
        row_speed.addWidget(lbl_j)
        self._jog_slow = QRadioButton("Медленно")
        self._jog_norm = QRadioButton("Обычно")
        self._jog_fast = QRadioButton("Быстро")
        self._jog_norm.setChecked(True)
        grp = QButtonGroup(self)
        for rb in (self._jog_slow, self._jog_norm, self._jog_fast):
            grp.addButton(rb)
            row_speed.addWidget(rb)
        row_speed.addStretch(1)
        root.addLayout(row_speed)

        row_jog = QHBoxLayout()
        b_ccw = QPushButton("⟲ Против")
        b_cw = QPushButton("⟳ По час.")
        b_ccw.pressed.connect(lambda: self._jog_start("-"))
        b_ccw.released.connect(self._jog_stop)
        b_cw.pressed.connect(lambda: self._jog_start("+"))
        b_cw.released.connect(self._jog_stop)
        tip = "Удерживайте — вращение, отпустите — стоп"
        b_ccw.setToolTip(tip)
        b_cw.setToolTip(tip)
        b_ccw.setMinimumWidth(80)
        b_cw.setMinimumWidth(80)
        row_jog.addWidget(b_ccw, 1)
        row_jog.addWidget(b_cw, 1)
        root.addLayout(row_jog)

        self._jog_restored = False

    def eventFilter(self, obj, ev) -> bool:  # noqa: N802
        if obj is self._target and isinstance(ev, QKeyEvent):
            if ev.type() == ev.Type.KeyPress and ev.key() in (
                    Qt.Key.Key_Return, Qt.Key.Key_Enter):
                self._go()
                return True
        return super().eventFilter(obj, ev)

    def _jog_speed(self) -> float:
        if self._jog_slow.isChecked():
            return P.JOG_SLOW_DEG_S
        if self._jog_fast.isChecked():
            return P.JOG_FAST_DEG_S
        return self._speed.value()

    def _jog_start(self, sign: str) -> None:
        self._jog_restored = False
        if self._jog_norm.isChecked():
            self._send(P.cmd_jog(sign))
        else:
            self._send(P.cmd_speed(self._jog_speed()))
            self._send(P.cmd_jog(sign))

    def _jog_stop(self) -> None:
        self._send(P.cmd_stop())
        if not self._jog_norm.isChecked() and not self._jog_restored:
            self._send(P.cmd_speed(self._speed.value()))
            self._jog_restored = True

    def apply_state(self, st: DeviceState) -> None:
        self._row_v.set_confirmed(st.v)
        self._row_a.set_confirmed(st.a)
        if st.tp is not None:
            txt = f"= {st.tp:.2f}°"
            self._echo_tp.setText(txt)
            if abs(self._target.value() - st.tp) > 0.05:
                self._echo_tp.setStyleSheet(f"color: {COLORS['mismatch']};")
            else:
                self._echo_tp.setStyleSheet(f"color: {COLORS['text_dim']};")
        else:
            self._echo_tp.setText("—")
            self._echo_tp.setStyleSheet("")

    def reset(self) -> None:
        self._row_v.reset()
        self._row_a.reset()
        self._echo_tp.setText("—")
        self._echo_tp.setStyleSheet("")

    def _go(self) -> None:
        self._send(P.cmd_target(self._target.value()))

    def set_enabled_controls(self, on: bool) -> None:
        for w in self.findChildren(QPushButton):
            w.setEnabled(on)
        for w in self.findChildren(QDoubleSpinBox):
            w.setEnabled(on)
        for w in self.findChildren(QRadioButton):
            w.setEnabled(on)
