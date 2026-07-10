"""TelemetryPlot — живые графики телеметрии (pyqtgraph).

Кривые: cp (текущий угол), tp (цель), pe (ошибка), u (управление).
Скользящее окно по времени, пауза, очистка, экспорт CSV.
Одна строка телеметрии = одна точка (поля tp/pe/u есть только при debug=1).
"""
from __future__ import annotations

import csv
from collections import deque

from PySide6.QtCore import QElapsedTimer, Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (QFileDialog, QHBoxLayout, QLabel, QPushButton,
                               QVBoxLayout, QWidget)

from ..theme import COLORS

try:
    import pyqtgraph as pg
except ImportError:                     # pragma: no cover
    pg = None

WINDOW_S = 60.0
_REDRAW_MS = 50


class TelemetryPlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._rows: deque[dict] = deque()
        self._clock = QElapsedTimer()
        self._paused = False
        self._dirty = False

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 6, 8, 6)
        root.setSpacing(4)

        bar = QHBoxLayout()
        title = QLabel("ТЕЛЕМЕТРИЯ ВО ВРЕМЕНИ")
        # Единый стиль заголовков секций (как QGroupBox::title слева)
        title.setStyleSheet(
            f'color:{COLORS["accent"]}; font-size:11px; font-weight:700; '
            "letter-spacing:1px; background:transparent;")
        # Легенда — вне поля построения, в одной строке с заголовком:
        # ничего не перекрывает данные (см. ревью дизайна)
        legend = QLabel(
            f'<span style="color:{COLORS["cp"]}">━ cp текущий</span>&nbsp;&nbsp;&nbsp;'
            f'<span style="color:{COLORS["tp"]}">╍ tp цель</span>&nbsp;&nbsp;&nbsp;'
            f'<span style="color:{COLORS["pe"]}">━ pe ошибка</span>&nbsp;&nbsp;&nbsp;'
            f'<span style="color:{COLORS["u"]}">━ u управление</span>')
        legend.setStyleSheet("font-size:11px; background:transparent;")
        legend.setTextFormat(Qt.RichText)
        self._btn_pause = QPushButton("Пауза")
        self._btn_pause.setCheckable(True)
        self._btn_pause.toggled.connect(self._on_pause)
        self._btn_clear = QPushButton("Очистить")
        self._btn_clear.clicked.connect(self.clear)
        self._btn_csv = QPushButton("Экспорт CSV")
        self._btn_csv.clicked.connect(self._export_csv)
        bar.addWidget(title)
        bar.addSpacing(18)
        bar.addWidget(legend)
        bar.addStretch(1)
        bar.addWidget(self._btn_pause)
        bar.addWidget(self._btn_clear)
        bar.addWidget(self._btn_csv)
        root.addLayout(bar)

        if pg is None:
            root.addWidget(QLabel("Не установлен pyqtgraph: pip install pyqtgraph"))
            self._plot = None
            return

        self._plot = pg.PlotWidget()
        self._plot.setBackground(COLORS["well"])
        self._plot.setLabel("bottom", "Время", units="с")
        self._plot.setLabel("left", "Угол / упр.", units="°")
        # Оси: тонкое перо, приглушённый текст; сетку включаем ПОСЛЕ пера —
        # линии сетки рисуются пером оси с альфой grid
        for name in ("left", "bottom"):
            ax = self._plot.getAxis(name)
            ax.setPen(pg.mkPen("#56637a"))
            ax.setTextPen(pg.mkPen(COLORS["text_dim"]))
            ax.enableAutoSIPrefix(False)   # без «m°» на пустом графике
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        # Нулевая линия Y — сильнее сетки: pe и u осциллируют вокруг нуля
        zero = pg.InfiniteLine(pos=0.0, angle=0,
                               pen=pg.mkPen(QColor(255, 255, 255, 40), width=1))
        self._plot.addItem(zero)
        self._plot.setDownsampling(mode="peak")
        self._plot.setClipToView(True)
        root.addWidget(self._plot, 1)

        def _curve(color, width=2.0, dash=None, z=0):
            pen = pg.mkPen(color=color, width=width)
            if dash:
                pen.setDashPattern(dash)
            c = self._plot.plot([], [], pen=pen)
            c.setZValue(z)
            return c

        # cp рисуется ПОВЕРХ штриховой tp, чтобы текущий угол не терялся под целью
        self._c_u = _curve(COLORS["u"], width=1.5, z=1)
        self._c_pe = _curve(COLORS["pe"], width=1.4, z=2)
        self._c_tp = _curve(COLORS["tp"], width=1.8, dash=[4, 5], z=3)
        self._c_cp = _curve(COLORS["cp"], width=2.2, z=4)

        self._redraw = QTimer(self)
        self._redraw.setInterval(_REDRAW_MS)
        self._redraw.timeout.connect(self._refresh)
        self._redraw.start()

    # ── Данные ─────────────────────────────────────────────────────────────
    def add_sample(self, data: dict) -> None:
        if pg is None or self._paused or "cp" not in data:
            return
        if not self._clock.isValid():
            self._clock.start()
        t = self._clock.elapsed() / 1000.0
        row = {"t": t, "cp": data["cp"]}
        for k in ("tp", "pe", "u"):
            if k in data:
                row[k] = data[k]
        self._rows.append(row)
        # обрезка окна
        t_min = t - WINDOW_S
        while self._rows and self._rows[0]["t"] < t_min:
            self._rows.popleft()
        self._dirty = True

    def clear(self) -> None:
        self._rows.clear()
        self._dirty = True

    # ── Отрисовка ──────────────────────────────────────────────────────────
    def _refresh(self) -> None:
        if pg is None or not self._dirty:
            return
        self._dirty = False
        self._set(self._c_cp, "cp")
        self._set(self._c_tp, "tp")
        self._set(self._c_pe, "pe")
        self._set(self._c_u, "u")

    def _set(self, curve, key) -> None:
        xs, ys = [], []
        for r in self._rows:
            if key in r:
                xs.append(r["t"])
                ys.append(r[key])
        curve.setData(xs, ys)

    def _on_pause(self, on: bool) -> None:
        self._paused = on
        self._btn_pause.setText("Продолжить" if on else "Пауза")

    def _export_csv(self) -> None:
        if not self._rows:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Сохранить телеметрию",
                                              "telemetry.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            wr = csv.writer(f)
            wr.writerow(["t", "cp", "tp", "pe", "u"])
            for r in self._rows:
                wr.writerow([f"{r['t']:.3f}", r.get("cp", ""), r.get("tp", ""),
                             r.get("pe", ""), r.get("u", "")])
