"""HeaderBar — шапка приложения: бренд, чип подключения, режим и крупный угол."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QLinearGradient, QPainter, QPen
from PySide6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from ..theme import COLORS

_MONO = '"Cascadia Mono","Consolas"'


def _repolish(w: QLabel) -> None:
    """Перечитать QSS после смены динамического свойства (chip)."""
    st = w.style()
    st.unpolish(w)
    st.polish(w)


class HeaderBar(QWidget):
    """Верхняя полоса 54px: слева бренд, справа статус подключения и угол антенны.

    API:
        set_connection(connected, text) — чип подключения (ok/off);
        set_angle(deg | None)           — крупный угол антенны;
        set_mode(mode | None)           — чип режима: cl → зелёный, ol → янтарный.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("headerBar")
        self.setFixedHeight(54)

        root = QHBoxLayout(self)
        root.setContentsMargins(16, 6, 16, 6)
        root.setSpacing(12)

        # ── Бренд слева ────────────────────────────────────────────────────
        brand = QVBoxLayout()
        brand.setContentsMargins(0, 0, 0, 0)
        brand.setSpacing(0)
        title = QLabel("◉ SONAR DEBUG")
        title.setStyleSheet(
            f'color:{COLORS["accent"]}; font-family:{_MONO}; '
            "font-size:16px; font-weight:800; letter-spacing:2px; "
            "background:transparent;")
        subtitle = QLabel("FW_SonarMotorDriver · STM32F103")
        subtitle.setStyleSheet(
            f'color:{COLORS["text_faint"]}; font-size:11px; background:transparent;')
        brand.addWidget(title)
        brand.addWidget(subtitle)
        root.addLayout(brand)
        root.addStretch(1)

        # ── Чип режима (CL/OL); скрыт, пока нет телеметрии ─────────────────
        self._mode_chip = QLabel("")
        self._mode_chip.setProperty("chip", "off")
        self._mode_chip.hide()
        root.addWidget(self._mode_chip)

        # ── Чип подключения ────────────────────────────────────────────────
        self._conn_chip = QLabel("НЕ ПОДКЛЮЧЕНО")
        self._conn_chip.setProperty("chip", "off")
        root.addWidget(self._conn_chip)

        # ── Угол антенны — вторичный readout (главное число живёт на PPI) ──
        self._angle = QLabel("— °")
        self._angle.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._angle.setMinimumWidth(84)   # чтобы цифры не «дёргали» компоновку
        self._angle.setStyleSheet(
            f'color:{COLORS["text_dim"]}; font-family:{_MONO}; '
            "font-size:13px; font-weight:600; background:transparent;")
        root.addWidget(self._angle)

    # ── Публичный API ──────────────────────────────────────────────────────
    def set_connection(self, connected: bool, text: str,
                       warn: bool = False) -> None:
        """Чип канала: ok — зелёный (реальное железо), warn — янтарный
        (симулятор: зелёный зарезервирован за состояниями OK), off — серый."""
        self._conn_chip.setText(text)
        state = ("warn" if warn else "ok") if connected else "off"
        self._conn_chip.setProperty("chip", state)
        _repolish(self._conn_chip)

    def set_angle(self, deg: float | None) -> None:
        """Крупный угол антенны; None — прочерк (нет данных)."""
        self._angle.setText("— °" if deg is None else f"{deg % 360.0:.2f}°")

    def set_mode(self, mode: str | None) -> None:
        """Чип режима регулятора: cl → зелёная точка, ol → янтарная, None — скрыть."""
        if not mode:
            self._mode_chip.hide()
            return
        # Единый шаблон чипа режима «● КОД — РАСШИФРОВКА» (как на PPI и в телеметрии)
        m = mode.lower()
        if m == "cl":
            self._mode_chip.setText("● CL — ЗАМКНУТЫЙ")
            self._mode_chip.setProperty("chip", "ok")
        elif m == "ol":
            self._mode_chip.setText("● OL — ОТКРЫТЫЙ")
            self._mode_chip.setProperty("chip", "warn")
        else:
            self._mode_chip.setText(mode.upper())
            self._mode_chip.setProperty("chip", "off")
        _repolish(self._mode_chip)
        self._mode_chip.show()

    # ── Фон: вертикальный градиент + нижняя граница (не зависит от QSS) ────
    def paintEvent(self, ev) -> None:  # noqa: N802
        p = QPainter(self)
        grad = QLinearGradient(0.0, 0.0, 0.0, float(self.height()))
        grad.setColorAt(0.0, QColor("#141a26"))
        grad.setColorAt(1.0, QColor("#0e1219"))
        p.fillRect(self.rect(), grad)
        p.setPen(QPen(QColor(COLORS["border"]), 1))
        p.drawLine(0, self.height() - 1, self.width(), self.height() - 1)
