"""CollapsibleSection — сворачиваемая секция с запоминанием в QSettings."""
from __future__ import annotations

from PySide6.QtCore import QSettings, Qt
from PySide6.QtWidgets import (QFrame, QHBoxLayout, QPushButton, QSizePolicy,
                               QVBoxLayout, QWidget)


class CollapsibleSection(QWidget):
    """Заголовок-кнопка + содержимое; состояние сохраняется по settings_key."""

    def __init__(self, title: str, settings_key: str,
                 settings: QSettings, parent=None):
        super().__init__(parent)
        self._key = settings_key
        self._settings = settings
        self._expanded = bool(settings.value(settings_key, True, type=bool))

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self._toggle = QPushButton()
        self._toggle.setFlat(True)
        self._toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        self._toggle.clicked.connect(self._on_toggle)
        self._toggle.setSizePolicy(QSizePolicy.Policy.Expanding,
                                   QSizePolicy.Policy.Fixed)
        root.addWidget(self._toggle)

        self._body = QFrame()
        self._body.setFrameShape(QFrame.NoFrame)
        self._body_layout = QVBoxLayout(self._body)
        self._body_layout.setContentsMargins(0, 4, 0, 0)
        self._body_layout.setSpacing(0)
        root.addWidget(self._body)

        self._title = title
        self._sync()

    def content_layout(self) -> QVBoxLayout:
        return self._body_layout

    def set_expanded(self, on: bool) -> None:
        self._expanded = on
        self._sync()
        self._settings.setValue(self._key, on)

    def _on_toggle(self) -> None:
        self.set_expanded(not self._expanded)

    def _sync(self) -> None:
        chevron = "▼" if self._expanded else "▶"
        self._toggle.setText(f"{chevron}  {self._title}")
        self._body.setVisible(self._expanded)
