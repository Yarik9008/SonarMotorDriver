"""ConsolePanel — консоль команд: цвет TX/RX, история, автопрокрутка."""
from __future__ import annotations

from typing import Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QKeyEvent
from PySide6.QtWidgets import (QCheckBox, QGroupBox, QHBoxLayout, QLineEdit,
                               QPlainTextEdit, QPushButton, QVBoxLayout)

from ..theme import COLORS, mono_font

_PREFIX = {
    "TX":   ("→", COLORS["accent"]),
    "RX":   ("←", COLORS["green"]),
    "ERR":  ("✕", COLORS["danger"]),
    "INFO": ("·", COLORS["text_faint"]),
}
_MAX_BLOCKS = 5000


class ConsolePanel(QGroupBox):
    def __init__(self, send_raw: Callable[[str], bool], parent=None):
        super().__init__("КОНСОЛЬ", parent)
        self._send_raw = send_raw
        self._history: list[str] = []
        self._hist_idx = -1
        self._draft = ""

        root = QVBoxLayout(self)

        bar = QHBoxLayout()
        self._autoscroll = QCheckBox("Автопрокрутка")
        self._autoscroll.setChecked(True)
        bar.addWidget(self._autoscroll)
        bar.addStretch(1)
        root.addLayout(bar)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(_MAX_BLOCKS)
        self._log.setFont(mono_font(10))
        root.addWidget(self._log, 1)

        row = QHBoxLayout()
        self._input = QLineEdit()
        self._input.setFont(mono_font(10))
        self._input.setPlaceholderText("Команда, напр. t=90 / scan=0,180,10,100 / mcfg")
        self._input.returnPressed.connect(self._send)
        self._input.installEventFilter(self)
        b_send = QPushButton("Отправить")
        b_send.clicked.connect(self._send)
        b_clear = QPushButton("Очистить")
        b_clear.clicked.connect(self._log.clear)
        row.addWidget(self._input, 1)
        row.addWidget(b_send)
        row.addWidget(b_clear)
        root.addLayout(row)

    def eventFilter(self, obj, ev) -> bool:  # noqa: N802
        if obj is not self._input or not isinstance(ev, QKeyEvent):
            return super().eventFilter(obj, ev)
        if ev.type() != ev.Type.KeyPress:
            return super().eventFilter(obj, ev)
        if ev.key() == Qt.Key.Key_Up and self._history:
            if self._hist_idx < 0:
                self._draft = self._input.text()
                self._hist_idx = len(self._history)
            if self._hist_idx > 0:
                self._hist_idx -= 1
                self._input.setText(self._history[self._hist_idx])
            return True
        if ev.key() == Qt.Key.Key_Down and self._hist_idx >= 0:
            self._hist_idx += 1
            if self._hist_idx >= len(self._history):
                self._hist_idx = -1
                self._input.setText(self._draft)
            else:
                self._input.setText(self._history[self._hist_idx])
            return True
        return super().eventFilter(obj, ev)

    def _send(self) -> None:
        text = self._input.text().strip()
        if not text:
            return
        self._send_raw(text)
        if not self._history or self._history[-1] != text:
            self._history.append(text)
        self._hist_idx = -1
        self._draft = ""
        self._input.clear()

    def append_log(self, ts: str, kind: str, text: str) -> None:
        prefix, color = _PREFIX.get(kind, ("·", COLORS["text"]))
        safe = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        label = kind.ljust(4).replace(" ", "&nbsp;")
        sb = self._log.verticalScrollBar()
        at_bottom = sb.value() >= sb.maximum() - 4
        self._log.appendHtml(
            f'<span style="color:{COLORS["text_faint"]}">{ts}</span>&nbsp;'
            f'<span style="color:{color}">{prefix}&nbsp;{label}&nbsp;{safe}</span>')
        if self._autoscroll.isChecked() or at_bottom:
            sb.setValue(sb.maximum())

    def set_enabled_controls(self, on: bool) -> None:
        self._input.setEnabled(on)
