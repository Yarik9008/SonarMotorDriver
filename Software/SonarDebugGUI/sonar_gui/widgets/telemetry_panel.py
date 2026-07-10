"""TelemetryPanel — период телеметрии, debug и числовые показания (без дублей cp/режима)."""
from __future__ import annotations

from typing import Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import (QCheckBox, QGridLayout, QGroupBox, QHBoxLayout,
                               QLabel, QPushButton, QSpinBox, QVBoxLayout)

from .. import protocol as P
from ..device_state import DeviceState
from ..metrics import METRICS, label
from ..theme import COLORS, metric_color, mono_font


def _set_chip(lbl: QLabel, state: str, text: str) -> None:
    lbl.setText(text)
    lbl.setProperty("chip", state)
    st = lbl.style()
    st.unpolish(lbl)
    st.polish(lbl)


class TelemetryPanel(QGroupBox):
    def __init__(self, send: Callable[[str], bool], parent=None):
        super().__init__("ТЕЛЕМЕТРИЯ", parent)
        self._send = send
        root = QVBoxLayout(self)
        root.setSpacing(6)

        row = QHBoxLayout()
        m_op = METRICS["op"]
        lbl_op = QLabel(label("op"))
        lbl_op.setProperty("dim", "true")
        lbl_op.setToolTip(m_op.tooltip)
        row.addWidget(lbl_op)
        self._op = QSpinBox()
        self._op.setRange(P.OP_MIN, P.OP_MAX)
        self._op.setValue(P.DEFAULTS.op_ms)
        row.addWidget(self._op, 1)
        b_op = QPushButton("✓")
        b_op.setFixedWidth(34)
        b_op.setToolTip("Применить период (Enter в поле)")
        b_op.clicked.connect(self._apply_op)
        self._op.installEventFilter(self)
        row.addWidget(b_op)
        self._echo_op = QLabel("—")
        self._echo_op.setProperty("dim", "true")
        self._echo_op.setFont(mono_font(12))
        self._echo_op.setMinimumWidth(56)
        row.addWidget(self._echo_op)
        self._debug = QCheckBox("Полная (debug=1)")
        self._debug.setToolTip("Полная телеметрия: ошибка, управление, PID, потери кадров")
        self._debug.toggled.connect(lambda on: self._send(P.cmd_debug(on)))
        row.addWidget(self._debug)
        root.addLayout(row)

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        self._val: dict[str, QLabel] = {}
        for i, key in enumerate(("tp", "pe", "u")):
            r, col = divmod(i, 2)
            m = METRICS[key]
            cap = QLabel(label(key) if key != "u" else m.label)
            cap.setProperty("dim", "true")
            cap.setToolTip(m.tooltip)
            val = QLabel("—")
            val.setFont(mono_font(13))
            val.setStyleSheet(f"color: {metric_color(key)};")
            grid.addWidget(cap, r, col * 2)
            grid.addWidget(val, r, col * 2 + 1)
            self._val[key] = val
        root.addLayout(grid)

        row_st = QHBoxLayout()
        row_st.setSpacing(6)
        self._ec = QLabel()
        _set_chip(self._ec, "off", f"{METRICS['ec'].short} —")
        self._ec.setToolTip(METRICS["ec"].tooltip)
        self._drp = QLabel(f"{METRICS['drp'].short} —")
        self._drp.setProperty("dim", "true")
        self._drp.setFont(mono_font(12, bold=False))
        self._drp.setToolTip(METRICS["drp"].tooltip)
        row_st.addWidget(self._ec)
        row_st.addStretch(1)
        row_st.addWidget(self._drp)
        root.addLayout(row_st)

    def _apply_op(self) -> None:
        self._send(P.cmd_op(self._op.value()))

    def eventFilter(self, obj, ev) -> bool:  # noqa: N802
        if obj is self._op and isinstance(ev, QKeyEvent):
            if ev.type() == ev.Type.KeyPress and ev.key() in (
                    Qt.Key.Key_Return, Qt.Key.Key_Enter):
                self._apply_op()
                return True
        return super().eventFilter(obj, ev)

    def apply_state(self, st: DeviceState) -> None:
        for key, fmt in (("tp", "{:.2f}"), ("pe", "{:.2f}"), ("u", "{:.4f}")):
            v = getattr(st, key)
            self._val[key].setText("—" if v is None else fmt.format(v))
        if st.drp is not None:
            self._drp.setText(f"{METRICS['drp'].short} {st.drp}")
        else:
            self._drp.setText(f"{METRICS['drp'].short} —")
        if st.ec is not None:
            ec = st.ec
            _set_chip(self._ec, "ok" if ec == 0 else "err",
                      f"{METRICS['ec'].short} {ec} · {P.EC_LEGEND.get(ec, '?')}")
        else:
            _set_chip(self._ec, "off", f"{METRICS['ec'].short} —")
        if st.op_ms is not None:
            self._echo_op.setText(f"= {st.op_ms}")
            if self._op.value() != st.op_ms:
                self._echo_op.setStyleSheet(f"color: {COLORS['mismatch']};")
            else:
                self._echo_op.setStyleSheet(f"color: {COLORS['text_dim']};")
        else:
            self._echo_op.setText("—")
            self._echo_op.setStyleSheet("")

    def reset(self) -> None:
        for v in self._val.values():
            v.setText("—")
        self._drp.setText(f"{METRICS['drp'].short} —")
        _set_chip(self._ec, "off", f"{METRICS['ec'].short} —")
        self._echo_op.setText("—")
        self._echo_op.setStyleSheet("")

    def set_enabled_controls(self, on: bool) -> None:
        self._op.setEnabled(on)
        self._debug.setEnabled(on)
