"""PidPanel — коэффициенты PID (kp/ki/kd) с эхо-значениями из телеметрии."""
from __future__ import annotations

from typing import Callable

from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QDoubleSpinBox, QGridLayout, QGroupBox, QLabel,
                               QPushButton)

from .. import protocol as P


def _mono(px: int) -> QFont:
    f = QFont()
    f.setFamilies(["Cascadia Mono", "Consolas", "monospace"])
    f.setPixelSize(px)
    return f


class PidPanel(QGroupBox):
    def __init__(self, send: Callable[[str], bool], parent=None):
        super().__init__("РЕГУЛЯТОР PID", parent)
        self._send = send
        d = P.DEFAULTS
        grid = QGridLayout(self)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        grid.setColumnStretch(1, 1)          # спинбокс тянется, панель не распирает

        self._spins: dict[str, QDoubleSpinBox] = {}
        self._echo: dict[str, QLabel] = {}
        for row, (key, label, val, builder) in enumerate([
            ("kp", "Kp", d.kp, P.cmd_kp),
            ("ki", "Ki", d.ki, P.cmd_ki),
            ("kd", "Kd", d.kd, P.cmd_kd),
        ]):
            cap = QLabel(label)
            cap.setProperty("dim", "true")
            grid.addWidget(cap, row, 0)
            sp = QDoubleSpinBox()
            sp.setRange(0.0, 1000.0)
            sp.setDecimals(4)
            sp.setSingleStep(0.005)
            sp.setValue(val)
            grid.addWidget(sp, row, 1)
            btn = QPushButton("✓")
            btn.setFixedWidth(34)
            btn.setToolTip(f"Применить {label}")
            btn.clicked.connect(
                lambda _=False, b=builder, s=sp: self._send(b(s.value())))
            grid.addWidget(btn, row, 2)
            echo = QLabel("—")                        # эхо фактического значения
            echo.setProperty("dim", "true")
            echo.setFont(_mono(12))
            echo.setMinimumWidth(72)
            grid.addWidget(echo, row, 3)
            self._spins[key] = sp
            self._echo[key] = echo

    def update_from_telemetry(self, data: dict) -> None:
        for key in ("kp", "ki", "kd"):
            if key in data:
                self._echo[key].setText(f"= {data[key]:.4f}")
