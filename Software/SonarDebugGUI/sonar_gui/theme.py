"""Тёмная тема «Sonar Ops Console»: палитра, типографика, отступы, QSS."""
from __future__ import annotations

from pathlib import Path

from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import QLabel

# ── Палитра ────────────────────────────────────────────────────────────────
# Сдержанная тёмная тема: приглушённые сигнальные цвета вместо ярких
# «неоновых» тонов — акценты остаются читаемыми, но не бросаются в глаза.
COLORS = {
    "bg":         "#0a0d13",
    "surface":    "#10141c",
    "surface2":   "#161c27",
    "well":       "#0b0f16",
    "border":     "#1f2735",
    "border2":    "#33405a",
    "text":       "#e4e7ee",
    "text_dim":   "#8f9aad",
    "text_faint": "#5c6880",
    "accent":     "#5aa6c2",
    "accent_dim": "#2a5568",
    "accent2":    "#57a397",
    "amber":      "#c99a5b",
    "danger":     "#c66a64",
    "green":      "#5faa82",
    "magenta":    "#a58bbd",
    "grid":       "#1a2230",
    "warn":       "#c99a5b",
    "cp": "#5aa6c2",
    "tp": "#c99a5b",
    "pe": "#a58bbd",
    "u":  "#8288b0",
    "ppi_center": "#0c2530",
    "ppi_edge":   "#070b10",
    "ppi_grid":   "#1c3944",
    "ppi_label":  "#6f97a8",
    "mismatch":   "#c99a5b",
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


def set_chip(label: QLabel, state: str, text: str | None = None) -> None:
    """Обновить текст/состояние QLabel с property chip=off/ok/warn/err (см. dark.qss)."""
    if text is not None:
        label.setText(text)
    label.setProperty("chip", state)
    st = label.style()
    st.unpolish(label)
    st.polish(label)


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
