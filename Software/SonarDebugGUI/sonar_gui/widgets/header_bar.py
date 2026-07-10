"""HeaderBar — шапка: бренд, чип подключения, единственный бейдж режима."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QLinearGradient, QPainter, QPen
from PySide6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from ..device_state import DeviceState
from ..metrics import mode_label
from ..theme import COLORS, FONT_MONO, SPACE_LG


def _repolish(w: QLabel) -> None:
    st = w.style()
    st.unpolish(w)
    st.polish(w)


class HeaderBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("headerBar")
        self.setFixedHeight(54)

        root = QHBoxLayout(self)
        root.setContentsMargins(SPACE_LG, 6, SPACE_LG, 6)
        root.setSpacing(12)

        brand = QVBoxLayout()
        brand.setContentsMargins(0, 0, 0, 0)
        brand.setSpacing(0)
        title = QLabel("◉ SONAR DEBUG")
        title.setStyleSheet(
            f'color:{COLORS["accent"]}; font-family:{FONT_MONO}; '
            "font-size:16px; font-weight:800; letter-spacing:2px; "
            "background:transparent;")
        subtitle = QLabel("FW_SonarMotorDriver · STM32F103")
        subtitle.setStyleSheet(
            f'color:{COLORS["text_faint"]}; font-size:11px; background:transparent;')
        brand.addWidget(title)
        brand.addWidget(subtitle)
        root.addLayout(brand)
        root.addStretch(1)

        self._mode_chip = QLabel("")
        self._mode_chip.setProperty("chip", "off")
        self._mode_chip.hide()
        root.addWidget(self._mode_chip)

        self._conn_chip = QLabel("НЕ ПОДКЛЮЧЕНО")
        self._conn_chip.setProperty("chip", "off")
        self._conn_chip.setMinimumWidth(120)
        self._conn_chip.setToolTip("Состояние канала связи")
        root.addWidget(self._conn_chip)

    def apply_state(self, st: DeviceState) -> None:
        if st.connected:
            state = "warn" if st.channel_warn else "ok"
            self._conn_chip.setText(st.channel)
            self._conn_chip.setToolTip(st.channel)
        else:
            state = "off"
            self._conn_chip.setText(st.channel or "НЕ ПОДКЛЮЧЕНО")
            self._conn_chip.setToolTip("")
        self._conn_chip.setProperty("chip", state)
        _repolish(self._conn_chip)

        if st.connected and st.has_telemetry and st.m:
            m = st.m.lower()
            if m == "cl":
                self._mode_chip.setText(mode_label("cl"))
                self._mode_chip.setProperty("chip", "ok")
            elif m == "ol":
                self._mode_chip.setText(mode_label("ol"))
                self._mode_chip.setProperty("chip", "warn")
            else:
                self._mode_chip.setText(st.m.upper())
                self._mode_chip.setProperty("chip", "off")
            _repolish(self._mode_chip)
            self._mode_chip.show()
        else:
            self._mode_chip.hide()

    def paintEvent(self, ev) -> None:  # noqa: N802
        p = QPainter(self)
        grad = QLinearGradient(0.0, 0.0, 0.0, float(self.height()))
        grad.setColorAt(0.0, QColor("#141a26"))
        grad.setColorAt(1.0, QColor("#0e1219"))
        p.fillRect(self.rect(), grad)
        p.setPen(QPen(QColor(COLORS["border"]), 1))
        p.drawLine(0, self.height() - 1, self.width(), self.height() - 1)
