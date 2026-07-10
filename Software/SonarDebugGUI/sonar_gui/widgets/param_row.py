"""ParamRow — паттерн «введено → применено» с эхо подтверждённого значения."""
from __future__ import annotations

from typing import Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import (QAbstractSpinBox, QHBoxLayout, QLabel,
                               QPushButton, QWidget)

from ..theme import COLORS, mono_font


class ParamRow(QWidget):
    """Поле ввода + кнопка применения + серое эхо; расхождение подсвечивается."""

    def __init__(self, caption: str, editor: QAbstractSpinBox,
                 apply_fn: Callable[[], None], tooltip: str = "",
                 parent=None):
        super().__init__(parent)
        self._editor = editor
        self._apply_fn = apply_fn
        self._confirmed: float | int | None = None
        self._fmt = "{:.4f}"

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)

        cap = QLabel(caption)
        cap.setProperty("dim", "true")
        if tooltip:
            cap.setToolTip(tooltip)
            editor.setToolTip(tooltip)
        row.addWidget(cap)
        row.addWidget(editor, 1)

        btn = QPushButton("✓")
        btn.setFixedWidth(34)
        btn.setToolTip("Применить (Enter)")
        btn.clicked.connect(apply_fn)
        row.addWidget(btn)

        self._echo = QLabel("—")
        self._echo.setProperty("dim", "true")
        self._echo.setFont(mono_font(12))
        self._echo.setMinimumWidth(72)
        self._echo.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        row.addWidget(self._echo)

        editor.valueChanged.connect(self._check_mismatch)
        editor.installEventFilter(self)

    def eventFilter(self, obj, ev) -> bool:  # noqa: N802
        if obj is self._editor and isinstance(ev, QKeyEvent):
            if ev.type() == ev.Type.KeyPress and ev.key() in (
                    Qt.Key.Key_Return, Qt.Key.Key_Enter):
                self._apply_fn()
                return True
        return super().eventFilter(obj, ev)

    def set_format(self, fmt: str) -> None:
        self._fmt = fmt

    def set_confirmed(self, value: float | int | None) -> None:
        self._confirmed = value
        if value is None:
            self._echo.setText("—")
            self._echo.setStyleSheet("")
            return
        self._echo.setText(f"= {self._fmt.format(value)}")
        self._check_mismatch()

    def _check_mismatch(self) -> None:
        if self._confirmed is None:
            self._echo.setStyleSheet("")
            return
        cur = float(self._editor.value())
        diff = abs(cur - float(self._confirmed))
        tol = max(1e-6, abs(float(self._confirmed)) * 1e-4)
        if diff > tol:
            self._echo.setStyleSheet(f"color: {COLORS['mismatch']};")
        else:
            self._echo.setStyleSheet(f"color: {COLORS['text_dim']};")

    def reset(self) -> None:
        self.set_confirmed(None)
