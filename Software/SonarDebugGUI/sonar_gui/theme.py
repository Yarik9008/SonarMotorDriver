"""Тёмная тема «Sonar Ops Console»: палитра, типографика, отступы, QSS."""
from __future__ import annotations

from pathlib import Path

from PySide6.QtGui import QColor, QFont

# ── Палитра ────────────────────────────────────────────────────────────────
COLORS = {
    "bg":         "#0a0d13",
    "surface":    "#10141c",
    "surface2":   "#161c27",
    "well":       "#0b0f16",
    "border":     "#1f2735",
    "border2":    "#33405a",
    "text":       "#e8ecf4",
    "text_dim":   "#93a0b4",
    "text_faint": "#5c6880",
    "accent":     "#37d6ff",
    "accent_dim": "#0a5d7a",
    "accent2":    "#35e0c8",
    "amber":      "#ffb454",
    "danger":     "#ff5d5d",
    "green":      "#3ddc84",
    "magenta":    "#e879f9",
    "grid":       "#1a2230",
    "warn":       "#ffb454",
    "cp": "#37d6ff",
    "tp": "#ffb454",
    "pe": "#e879f9",
    "u":  "#9d8cff",
    "ppi_center": "#0c2530",
    "ppi_edge":   "#070b10",
    "ppi_grid":   "#164252",
    "ppi_label":  "#5fb8d4",
    "mismatch":   "#ffb454",
}

# ── Типографика (3 размера) ────────────────────────────────────────────────
FONT_UI = '"Segoe UI", "Inter", sans-serif'
FONT_MONO = '"Cascadia Mono", "Consolas", monospace'
FONT_SIZE_SM = 11
FONT_SIZE_MD = 13
FONT_SIZE_LG = 16

# ── Отступы (кратно 4 px) ──────────────────────────────────────────────────
SPACE_XS = 4
SPACE_SM = 8
SPACE_MD = 12
SPACE_LG = 16

# ── Радиусы ────────────────────────────────────────────────────────────────
RADIUS_SM = 6
RADIUS_MD = 7
RADIUS_LG = 10


def c(name: str) -> QColor:
    return QColor(COLORS[name])


def mono_font(px: int = FONT_SIZE_MD, *, bold: bool = False) -> QFont:
    f = QFont()
    f.setFamilies(["Cascadia Mono", "Consolas", "monospace"])
    f.setPixelSize(px)
    if bold:
        f.setWeight(QFont.Weight.DemiBold)
    return f


def ui_font(px: int = FONT_SIZE_MD, *, bold: bool = False) -> QFont:
    f = QFont("Segoe UI")
    f.setPixelSize(px)
    if bold:
        f.setWeight(QFont.Weight.DemiBold)
    return f


def metric_color(key: str) -> str:
    return COLORS.get(key, COLORS["text_dim"])


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
