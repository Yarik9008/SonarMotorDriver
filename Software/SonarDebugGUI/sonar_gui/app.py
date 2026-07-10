"""Точка сборки приложения: QApplication + тёмная тема + иконка + главное окно."""
from __future__ import annotations

import math
import sys

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QIcon, QPainter, QPen, QPixmap, QRadialGradient
from PySide6.QtWidgets import QApplication

from . import theme
from .main_window import MainWindow


def _make_window_icon() -> QIcon:
    """Иконка окна: тёмный диск сонара, циановое кольцо, луч-клин и точка на ободе.

    Рисуется QPainter'ом (64×64), чтобы не тащить файлы ресурсов.
    Цвета — из палитры спецификации (фиксированные, не зависят от theme.COLORS).
    """
    pm = QPixmap(64, 64)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing, True)

    cx = cy = 32.0
    accent = QColor("#37d6ff")

    # Тёмный диск с радиальным градиентом и ободом
    grad = QRadialGradient(QPointF(cx, cy), 30.0)
    grad.setColorAt(0.0, QColor("#0c2530"))
    grad.setColorAt(1.0, QColor("#070b10"))
    p.setPen(QPen(QColor("#33405a"), 2.0))
    p.setBrush(grad)
    p.drawEllipse(QRectF(3, 3, 58, 58))

    # Циановое кольцо
    p.setBrush(Qt.NoBrush)
    p.setPen(QPen(accent, 2.5))
    p.drawEllipse(QRectF(8, 8, 48, 48))

    # Луч-клин: сектор 50°..90° (вверх-вправо), полупрозрачная заливка
    beam = QColor(accent)
    beam.setAlpha(80)
    p.setPen(Qt.NoPen)
    p.setBrush(beam)
    p.drawPie(QRectF(8, 8, 48, 48), 50 * 16, 40 * 16)

    # Осевая линия луча и точка на ободе (центр клина — 70°)
    a = math.radians(70.0)
    r = 24.0
    ex = cx + r * math.cos(a)
    ey = cy - r * math.sin(a)
    p.setPen(QPen(accent, 2.0))
    p.drawLine(QPointF(cx, cy), QPointF(ex, ey))
    p.setPen(Qt.NoPen)
    p.setBrush(accent)
    p.drawEllipse(QPointF(ex, ey), 3.5, 3.5)
    p.setBrush(QColor("#ffffff"))
    p.drawEllipse(QPointF(ex, ey), 1.5, 1.5)

    # Хаб в центре
    p.setBrush(accent)
    p.drawEllipse(QPointF(cx, cy), 2.5, 2.5)
    p.end()
    return QIcon(pm)


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("SonarDebugGUI")
    app.setOrganizationName("SonarMotorDriver")
    theme.apply(app)
    app.setWindowIcon(_make_window_icon())

    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
