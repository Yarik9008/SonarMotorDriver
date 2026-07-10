"""MotorPanel — мотор, PID-профиль, джог, драйвер TMC2209 и калибровка антенны."""
from __future__ import annotations

from typing import Callable

from PySide6.QtCore import QSettings, Qt, Signal
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import (QButtonGroup, QComboBox, QDoubleSpinBox,
                               QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                               QPushButton, QRadioButton, QSizePolicy,
                               QSpinBox, QVBoxLayout)

from .. import protocol as P
from ..device_state import DeviceState
from ..metrics import METRICS, label
from ..theme import COLORS, metric_color, mono_font, set_chip
from .param_row import ParamRow


class MotorPanel(QGroupBox):
    antenna_config_changed = Signal(float, float, int)   # scale, offset, sign

    def __init__(self, send: Callable[[str], bool], parent=None):
        super().__init__("МОТОР", parent)
        self._send = send
        self._s = QSettings("SonarMotorDriver", "SonarDebugGUI")
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
        b_go = QPushButton("Перейти")
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

        row_pe = QHBoxLayout()
        lbl_pe = QLabel(label("pe"))
        lbl_pe.setProperty("dim", "true")
        lbl_pe.setToolTip(METRICS["pe"].tooltip)
        self._pe_val = QLabel("—")
        self._pe_val.setFont(mono_font(13))
        self._pe_val.setStyleSheet(f"color: {metric_color('pe')};")
        row_pe.addWidget(lbl_pe)
        row_pe.addWidget(self._pe_val, 1)
        root.addLayout(row_pe)

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
        b_ccw = QPushButton("Против")
        b_cw = QPushButton("По час.")
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

        row_st = QHBoxLayout()
        row_st.setSpacing(6)
        self._ec = QLabel()
        set_chip(self._ec, "off", f"{METRICS['ec'].short} —")
        self._ec.setToolTip(METRICS["ec"].tooltip)
        self._drp = QLabel(f"{METRICS['drp'].short} —")
        self._drp.setProperty("dim", "true")
        self._drp.setFont(mono_font(12, bold=False))
        self._drp.setToolTip(METRICS["drp"].tooltip)
        row_st.addWidget(self._ec)
        row_st.addStretch(1)
        row_st.addWidget(self._drp)
        root.addLayout(row_st)

        # Драйвер TMC2209: токи движения/удержания, микрошаг, конфигурация (mcfg).
        lbl_drv = QLabel("Драйвер TMC2209")
        lbl_drv.setProperty("dim", "true")
        root.addWidget(lbl_drv)

        grid_drv = QGridLayout()
        grid_drv.setHorizontalSpacing(8)
        grid_drv.setColumnStretch(1, 1)
        self._irun = self._cur_spin(d.irun)
        self._ihold = self._cur_spin(d.ihold)

        cap_run = QLabel("I движ., мА")
        cap_run.setProperty("dim", "true")
        grid_drv.addWidget(cap_run, 0, 0)
        grid_drv.addWidget(self._irun, 0, 1)
        b_irun = QPushButton("✓")
        b_irun.setFixedWidth(34)
        b_irun.setToolTip("Применить ток движения (irun)")
        b_irun.clicked.connect(lambda: self._send(P.cmd_irun(self._irun.value())))
        grid_drv.addWidget(b_irun, 0, 2)

        cap_hold = QLabel("I удерж., мА")
        cap_hold.setProperty("dim", "true")
        grid_drv.addWidget(cap_hold, 1, 0)
        grid_drv.addWidget(self._ihold, 1, 1)
        b_ihold = QPushButton("✓")
        b_ihold.setFixedWidth(34)
        b_ihold.setToolTip("Применить ток удержания (ihold)")
        b_ihold.clicked.connect(lambda: self._send(P.cmd_ihold(self._ihold.value())))
        grid_drv.addWidget(b_ihold, 1, 2)
        root.addLayout(grid_drv)

        row_ms = QHBoxLayout()
        cap_ms = QLabel("Микрошаг")
        cap_ms.setProperty("dim", "true")
        row_ms.addWidget(cap_ms)
        self._mstep = QComboBox()
        for v in P.MSTEP_VALUES:
            self._mstep.addItem(str(v), v)
        self._mstep.setCurrentText(str(d.microsteps))
        row_ms.addWidget(self._mstep, 1)
        b_ms = QPushButton("✓")
        b_ms.setFixedWidth(34)
        b_ms.setToolTip("Применить микрошаг (mstep) — только на остановленном моторе")
        b_ms.clicked.connect(lambda: self._send(P.cmd_mstep(self._mstep.currentData())))
        row_ms.addWidget(b_ms)
        root.addLayout(row_ms)

        row_act = QHBoxLayout()
        b_icur = QPushButton("icur: оба")
        b_icur.setToolTip("Применить оба тока одной командой (icur)")
        b_icur.clicked.connect(
            lambda: self._send(P.cmd_icur(self._irun.value(), self._ihold.value())))
        b_cfg = QPushButton("Обновить (mcfg)")
        b_cfg.setToolTip("Запросить текущую конфигурацию драйвера")
        b_cfg.clicked.connect(lambda: self._send(P.cmd_mcfg()))
        b_icur.setMinimumWidth(80)
        b_cfg.setMinimumWidth(80)
        row_act.addWidget(b_icur, 1)
        row_act.addWidget(b_cfg, 1)
        root.addLayout(row_act)

        self._cfg = QLabel("mode=?  run=?  hold=?  microsteps=?  ready=?")
        self._cfg.setFont(mono_font(10))
        self._cfg.setWordWrap(True)
        self._cfg.setProperty("well", "true")
        root.addWidget(self._cfg)

        # Диаграмма антенны: пересчёт cp → угол антенны для PPI (только
        # локально в GUI, в прошивку не отправляется). Формула:
        # угол_антенны = sign · (cp · передаточное_число) + смещение_нуля
        lbl_ant = QLabel("Диаграмма антенны")
        lbl_ant.setProperty("dim", "true")
        root.addWidget(lbl_ant)

        grid_ant = QGridLayout()
        grid_ant.setHorizontalSpacing(8)
        grid_ant.setColumnStretch(1, 1)

        cap_scale = QLabel("Передаточное число")
        cap_scale.setProperty("dim", "true")
        grid_ant.addWidget(cap_scale, 0, 0)
        self._ant_scale = QDoubleSpinBox()
        self._ant_scale.setRange(0.0001, 10000.0)
        self._ant_scale.setDecimals(4)
        self._ant_scale.setValue(float(self._s.value("scale", 1.0)))
        self._ant_scale.setMinimumWidth(90)
        grid_ant.addWidget(self._ant_scale, 0, 1)

        cap_offset = QLabel("Смещение нуля, °")
        cap_offset.setProperty("dim", "true")
        grid_ant.addWidget(cap_offset, 1, 0)
        self._ant_offset = QDoubleSpinBox()
        self._ant_offset.setRange(-360.0, 360.0)
        self._ant_offset.setDecimals(2)
        self._ant_offset.setValue(float(self._s.value("offset", 0.0)))
        self._ant_offset.setMinimumWidth(90)
        grid_ant.addWidget(self._ant_offset, 1, 1)

        cap_sign = QLabel("Направление")
        cap_sign.setProperty("dim", "true")
        grid_ant.addWidget(cap_sign, 2, 0)
        self._ant_sign = QComboBox()
        self._ant_sign.addItem("+1 (по часовой)", 1)
        self._ant_sign.addItem("-1 (против)", -1)
        idx = self._ant_sign.findData(int(self._s.value("sign", 1)))
        self._ant_sign.setCurrentIndex(max(0, idx))
        self._ant_sign.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self._ant_sign.setMinimumContentsLength(12)
        self._ant_sign.setMinimumWidth(90)
        grid_ant.addWidget(self._ant_sign, 2, 1)

        root.addLayout(grid_ant)

        self._ant_scale.valueChanged.connect(self._emit_antenna)
        self._ant_offset.valueChanged.connect(self._emit_antenna)
        self._ant_sign.currentIndexChanged.connect(self._emit_antenna)

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

        self._pe_val.setText("—" if st.pe is None else f"{st.pe:.2f}°")

        if st.drp is not None:
            self._drp.setText(f"{METRICS['drp'].short} {st.drp}")
        else:
            self._drp.setText(f"{METRICS['drp'].short} —")
        if st.ec is not None:
            ec = st.ec
            set_chip(self._ec, "ok" if ec == 0 else "err",
                     f"{METRICS['ec'].short} {ec} · {P.EC_LEGEND.get(ec, '?')}")
        else:
            set_chip(self._ec, "off", f"{METRICS['ec'].short} —")

    def reset(self) -> None:
        self._row_v.reset()
        self._row_a.reset()
        self._echo_tp.setText("—")
        self._echo_tp.setStyleSheet("")
        self._pe_val.setText("—")
        self._drp.setText(f"{METRICS['drp'].short} —")
        set_chip(self._ec, "off", f"{METRICS['ec'].short} —")
        self._cfg.setText("mode=?  run=?  hold=?  microsteps=?  ready=?")

    def _go(self) -> None:
        self._send(P.cmd_target(self._target.value()))

    @staticmethod
    def _cur_spin(val) -> QSpinBox:
        sp = QSpinBox()
        sp.setRange(P.CURRENT_MIN, P.CURRENT_MAX)
        sp.setValue(val)
        return sp

    def update_mcfg(self, d: dict) -> None:
        self._cfg.setText(
            f"mode={d.get('mode', '?')}  run={d.get('run', '?')}  "
            f"hold={d.get('hold', '?')}  microsteps={d.get('microsteps', '?')}  "
            f"ready={d.get('ready', '?')}")

    def antenna_values(self) -> tuple[float, float, int]:
        return (self._ant_scale.value(), self._ant_offset.value(),
                int(self._ant_sign.currentData()))

    def _emit_antenna(self) -> None:
        scale, offset, sign = self.antenna_values()
        self._s.setValue("scale", scale)
        self._s.setValue("offset", offset)
        self._s.setValue("sign", sign)
        self.antenna_config_changed.emit(scale, offset, sign)

    def set_enabled_controls(self, on: bool) -> None:
        for w in self.findChildren(QPushButton):
            w.setEnabled(on)
        for w in self.findChildren(QDoubleSpinBox):
            w.setEnabled(on)
        for w in self.findChildren(QSpinBox):     # токи драйвера (irun/ihold)
            w.setEnabled(on)
        for w in self.findChildren(QRadioButton):
            w.setEnabled(on)
        self._mstep.setEnabled(on)                # микрошаг драйвера
        # Калибровка антенны — чисто локальная настройка GUI, в прошивку не
        # уходит, поэтому остаётся доступной независимо от подключения.
        self._ant_scale.setEnabled(True)
        self._ant_offset.setEnabled(True)
        self._ant_sign.setEnabled(True)
