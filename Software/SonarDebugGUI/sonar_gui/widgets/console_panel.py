"""ConsolePanel — сырой ввод команд (без валидации) и лог TX/RX с префиксами."""
from __future__ import annotations

from typing import Callable

from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QGroupBox, QHBoxLayout, QLineEdit, QPlainTextEdit,
                               QPushButton, QVBoxLayout)

from ..theme import COLORS

# Вид строки лога: моно-префикс вместо текстовой колонки kind.
#   → передано устройству, ← принято, ✕ ошибка, · служебное сообщение
_PREFIX = {
    "TX":   ("→", COLORS["accent"]),
    "RX":   ("←", COLORS["text"]),
    "ERR":  ("✕", COLORS["danger"]),
    "INFO": ("·", COLORS["text_faint"]),
}
_MAX_BLOCKS = 2000


def _mono_font(size: int = 10) -> QFont:
    font = QFont()
    font.setFamilies(["Cascadia Mono", "Consolas"])
    font.setPointSize(size)
    return font


class ConsolePanel(QGroupBox):
    def __init__(self, send_raw: Callable[[str], bool], parent=None):
        super().__init__("КОНСОЛЬ", parent)
        self._send_raw = send_raw
        root = QVBoxLayout(self)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(_MAX_BLOCKS)
        self._log.setFont(_mono_font(10))
        root.addWidget(self._log, 1)

        row = QHBoxLayout()
        self._input = QLineEdit()
        self._input.setFont(_mono_font(10))
        self._input.setPlaceholderText("Команда, напр. t=90 / scan=0,180,10,100 / mcfg")
        self._input.returnPressed.connect(self._send)
        b_send = QPushButton("Отправить")
        b_send.clicked.connect(self._send)
        b_clear = QPushButton("Очистить")
        b_clear.clicked.connect(self._log.clear)
        row.addWidget(self._input, 1)
        row.addWidget(b_send)
        row.addWidget(b_clear)
        root.addLayout(row)

    def _send(self) -> None:
        text = self._input.text().strip()
        if not text:
            return
        self._send_raw(text)
        self._input.clear()

    def append_log(self, kind: str, text: str) -> None:
        prefix, color = _PREFIX.get(kind, ("·", COLORS["text"]))
        safe = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        self._log.appendHtml(f'<span style="color:{color}">{prefix} {safe}</span>')
