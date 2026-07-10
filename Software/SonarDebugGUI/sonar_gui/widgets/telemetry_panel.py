"""TelemetryPanel — управление телеметрией (op/debug) и числовые показания."""
from __future__ import annotations

from typing import Callable

from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QCheckBox, QGridLayout, QGroupBox, QHBoxLayout,
                               QLabel, QPushButton, QSpinBox, QVBoxLayout)

from .. import protocol as P
from ..theme import COLORS


def _set_chip(label: QLabel, state: str, text: str | None = None) -> None:
    """Ставит чип-состояние (ok/warn/err/off) и заново применяет стиль."""
    if text is not None:
        label.setText(text)
    label.setProperty("chip", state)
    st = label.style()
    st.unpolish(label)
    st.polish(label)


def _mono(px: int, bold: bool = True) -> QFont:
    """Моноширинный шрифт показаний: Cascadia Mono с запасным Consolas."""
    f = QFont()
    f.setFamilies(["Cascadia Mono", "Consolas", "monospace"])
    f.setPixelSize(px)
    if bold:
        f.setWeight(QFont.Weight.DemiBold)
    return f


class TelemetryPanel(QGroupBox):
    def __init__(self, send: Callable[[str], bool], parent=None):
        super().__init__("ТЕЛЕМЕТРИЯ", parent)
        self._send = send
        root = QVBoxLayout(self)
        root.setSpacing(6)

        # Период телеметрии + флаг полной посылки
        row = QHBoxLayout()
        lbl_op = QLabel("op, мс")
        lbl_op.setProperty("dim", "true")
        row.addWidget(lbl_op)
        self._op = QSpinBox()
        self._op.setRange(P.OP_MIN, P.OP_MAX)
        self._op.setValue(P.DEFAULTS.op_ms)
        row.addWidget(self._op, 1)
        b_op = QPushButton("✓")
        b_op.setFixedWidth(34)
        b_op.setToolTip("Применить период телеметрии (op)")
        b_op.clicked.connect(lambda: self._send(P.cmd_op(self._op.value())))
        row.addWidget(b_op)
        self._debug = QCheckBox("debug")
        self._debug.setToolTip("debug=1 — полная телеметрия (pe, u, ec, drp)")
        self._debug.toggled.connect(lambda on: self._send(P.cmd_debug(on)))
        row.addWidget(self._debug)
        root.addLayout(row)

        # Показания парами в 2 колонки, моноширинные, с фиксированными цветами
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        self._val: dict[str, QLabel] = {}
        fields = [
            ("cp", "cp, °", COLORS["cp"]),
            ("tp", "tp, °", COLORS["tp"]),
            ("pe", "pe, °", COLORS["pe"]),
            ("u", "u", COLORS["text_dim"]),
        ]
        for i, (key, label, color) in enumerate(fields):
            r, col = divmod(i, 2)
            cap = QLabel(label)
            cap.setProperty("dim", "true")
            val = QLabel("—")
            val.setFont(_mono(13))
            val.setStyleSheet(f"color: {color};")
            grid.addWidget(cap, r, col * 2)
            grid.addWidget(val, r, col * 2 + 1)
            self._val[key] = val
        root.addLayout(grid)

        # Режим + код энкодера — чипами, счётчик drp — приглушённо
        row_st = QHBoxLayout()
        row_st.setSpacing(6)
        self._mode = QLabel()
        _set_chip(self._mode, "off", "РЕЖИМ —")
        self._ec = QLabel()
        _set_chip(self._ec, "off", "ec —")
        self._drp = QLabel("drp —")
        self._drp.setProperty("dim", "true")
        self._drp.setFont(_mono(12, bold=False))
        row_st.addWidget(self._mode)
        row_st.addWidget(self._ec)
        row_st.addStretch(1)
        row_st.addWidget(self._drp)
        root.addLayout(row_st)

    def update_from_telemetry(self, d: dict) -> None:
        if "cp" in d:
            self._val["cp"].setText(f"{d['cp']:.2f}")
        for key, fmt in (("tp", "{:.2f}"), ("pe", "{:.2f}"), ("u", "{:.4f}")):
            if key in d:
                self._val[key].setText(fmt.format(d[key]))
        if "drp" in d:
            self._drp.setText(f"drp {d['drp']}")
        if "ec" in d:
            ec = d["ec"]
            _set_chip(self._ec, "ok" if ec == 0 else "err",
                      f"ec {ec} · {P.EC_LEGEND.get(ec, '?')}")
        if "m" in d:
            # Единый шаблон чипа режима «● КОД — РАСШИФРОВКА» (как в шапке и на PPI)
            if d["m"] == "cl":
                _set_chip(self._mode, "ok", "● CL — ЗАМКНУТЫЙ")
            else:
                _set_chip(self._mode, "warn", "● OL — ОТКРЫТЫЙ")

    def reset(self) -> None:
        for v in self._val.values():
            v.setText("—")
        self._drp.setText("drp —")
        _set_chip(self._ec, "off", "ec —")
        _set_chip(self._mode, "off", "РЕЖИМ —")
