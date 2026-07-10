"""PidPanel — коэффициенты PID с эхо подтверждённых значений."""
from __future__ import annotations

from typing import Callable

from PySide6.QtWidgets import (QDoubleSpinBox, QGroupBox, QVBoxLayout)

from .. import protocol as P
from ..device_state import DeviceState
from ..widgets.param_row import ParamRow


class PidPanel(QGroupBox):
    def __init__(self, send: Callable[[str], bool], parent=None):
        super().__init__("РЕГУЛЯТОР PID", parent)
        self._send = send
        d = P.DEFAULTS
        root = QVBoxLayout(self)
        root.setSpacing(6)

        self._rows: dict[str, ParamRow] = {}
        for key, cap, val, builder in [
            ("kp", "Kp", d.kp, P.cmd_kp),
            ("ki", "Ki", d.ki, P.cmd_ki),
            ("kd", "Kd", d.kd, P.cmd_kd),
        ]:
            sp = QDoubleSpinBox()
            sp.setRange(0.0, 1000.0)
            sp.setDecimals(4)
            sp.setSingleStep(0.005)
            sp.setValue(val)
            row = ParamRow(cap, sp, lambda s=sp, b=builder: self._send(b(s.value())))
            row.set_format("{:.4f}")
            root.addWidget(row)
            self._rows[key] = row

    def apply_state(self, st: DeviceState) -> None:
        self._rows["kp"].set_confirmed(st.kp)
        self._rows["ki"].set_confirmed(st.ki)
        self._rows["kd"].set_confirmed(st.kd)

    def reset(self) -> None:
        for r in self._rows.values():
            r.reset()

    def set_enabled_controls(self, on: bool) -> None:
        for sp in self.findChildren(QDoubleSpinBox):
            sp.setEnabled(on)
