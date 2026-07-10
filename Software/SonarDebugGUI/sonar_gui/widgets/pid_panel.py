"""PidPanel — коэффициенты PID с эхо подтверждённых значений и выходом регулятора."""
from __future__ import annotations

from typing import Callable

from PySide6.QtWidgets import QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel, QVBoxLayout

from .. import protocol as P
from ..device_state import DeviceState
from ..metrics import METRICS, label
from ..theme import metric_color, mono_font
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

        row_u = QHBoxLayout()
        lbl_u = QLabel(label("u", with_unit=False))
        lbl_u.setProperty("dim", "true")
        lbl_u.setToolTip(METRICS["u"].tooltip)
        self._u_val = QLabel("—")
        self._u_val.setFont(mono_font(13))
        self._u_val.setStyleSheet(f"color: {metric_color('u')};")
        row_u.addWidget(lbl_u)
        row_u.addWidget(self._u_val, 1)
        root.addLayout(row_u)

    def apply_state(self, st: DeviceState) -> None:
        self._rows["kp"].set_confirmed(st.kp)
        self._rows["ki"].set_confirmed(st.ki)
        self._rows["kd"].set_confirmed(st.kd)
        self._u_val.setText("—" if st.u is None else f"{st.u:.4f}")

    def reset(self) -> None:
        for r in self._rows.values():
            r.reset()
        self._u_val.setText("—")

    def set_enabled_controls(self, on: bool) -> None:
        for sp in self.findChildren(QDoubleSpinBox):
            sp.setEnabled(on)
