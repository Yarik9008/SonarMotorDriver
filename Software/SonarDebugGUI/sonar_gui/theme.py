"""Тёмная тема «Sonar Ops Console»: палитра (виджеты + графики) и применение QSS."""
from __future__ import annotations

from pathlib import Path

from PySide6.QtGui import QColor

# Единая палитра — на неё опираются dark.qss, PPI-диаграмма и графики.
# Глубокий сине-чёрный фон, циановый «фосфор», тёплые/сигнальные акценты.
COLORS = {
    "bg":         "#0a0d13",   # окно
    "surface":    "#10141c",   # панели/группы
    "surface2":   "#161c27",   # приподнятые элементы, легенды
    "well":       "#0b0f16",   # «колодцы»: поля ввода, консоль, фон графика
    "border":     "#1f2735",
    "border2":    "#33405a",   # ховер/актив
    "text":       "#e8ecf4",
    "text_dim":   "#93a0b4",
    "text_faint": "#5c6880",
    "accent":     "#37d6ff",   # циан — основной
    "accent_dim": "#0a5d7a",   # заливка primary-кнопок
    "accent2":    "#35e0c8",   # тил — скан/вторичный
    "amber":      "#ffb454",
    "danger":     "#ff5d5d",
    "green":      "#3ddc84",
    "magenta":    "#e879f9",
    "grid":       "#1a2230",
    # Кривые/лучи телеметрии
    "cp": "#37d6ff",   # текущий угол
    "tp": "#ffb454",   # цель
    "pe": "#e879f9",   # ошибка
    "u":  "#9d8cff",   # управление — приглушённый фиолетовый, отличим от сетки/фона
    # PPI-диаграмма
    "ppi_center": "#0c2530",
    "ppi_edge":   "#070b10",
    "ppi_grid":   "#164252",
    "ppi_label":  "#5fb8d4",
}


def c(name: str) -> QColor:
    return QColor(COLORS[name])


def apply(app) -> None:
    """Применяет QSS ко всему приложению и настраивает pyqtgraph."""
    qss_path = Path(__file__).resolve().parent / "dark.qss"
    try:
        app.setStyleSheet(qss_path.read_text(encoding="utf-8"))
    except OSError:
        pass

    try:
        import pyqtgraph as pg
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption("background", COLORS["well"])
        pg.setConfigOption("foreground", COLORS["text_dim"])
    except Exception:                    # noqa: BLE001
        pass
