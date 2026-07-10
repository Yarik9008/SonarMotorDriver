"""TelemetryPlot — живые графики телеметрии (pyqtgraph).

Кривые: позиция, цель, ошибка (левая ось °), управление (правая ось).
"""
from __future__ import annotations

import csv
from collections import deque

from PySide6.QtCore import QElapsedTimer, Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (QFileDialog, QHBoxLayout, QLabel, QPushButton,
                               QVBoxLayout, QWidget)

from ..metrics import METRICS, legend_label
from ..theme import COLORS, metric_color

try:
    import pyqtgraph as pg
except ImportError:                     # pragma: no cover
    pg = None

WINDOW_S = 60.0
_REDRAW_MS = 50
_MARGIN_FRAC = 0.08
_PLOT_KEYS = ("cp", "tp", "pe", "u")


class TelemetryPlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._rows: deque[dict] = deque()
        self._clock = QElapsedTimer()
        self._paused = False
        self._dirty = False
        self._legend_labels: dict[str, QLabel] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 6, 8, 6)
        root.setSpacing(4)

        bar = QHBoxLayout()
        title = QLabel("ТЕЛЕМЕТРИЯ ВО ВРЕМЕНИ")
        title.setStyleSheet(
            f'color:{COLORS["accent"]}; font-size:11px; font-weight:700; '
            "letter-spacing:1px; background:transparent;")
        bar.addWidget(title)
        bar.addSpacing(12)

        legend_box = QHBoxLayout()
        legend_box.setSpacing(12)
        for key in _PLOT_KEYS:
            m = METRICS[key]
            sym = "╍" if key == "tp" else "━"
            lbl = QLabel(f'{sym} {legend_label(key)}')
            lbl.setStyleSheet(
                f"color:{metric_color(key)}; font-size:11px; background:transparent;")
            legend_box.addWidget(lbl)
            self._legend_labels[key] = lbl
        bar.addLayout(legend_box)
        bar.addStretch(1)

        self._btn_pause = QPushButton("Пауза")
        self._btn_pause.setCheckable(True)
        self._btn_pause.toggled.connect(self._on_pause)
        self._btn_clear = QPushButton("Очистить")
        self._btn_clear.clicked.connect(self.clear)
        self._btn_csv = QPushButton("Экспорт CSV")
        self._btn_csv.clicked.connect(self._export_csv)
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
        self._plot.setLabel("left", "Угол", units="°")
        self._plot.setLabel("right", METRICS["u"].label)

        self._vb_right = pg.ViewBox()
        self._plot.showAxis("right")
        self._plot.scene().addItem(self._vb_right)
        self._plot.getAxis("right").linkToView(self._vb_right)
        self._vb_right.setXLink(self._plot)

        for name in ("left", "bottom", "right"):
            ax = self._plot.getAxis(name)
            ax.setPen(pg.mkPen(COLORS["border2"]))
            ax.setTextPen(pg.mkPen(COLORS["text_dim"]))
            ax.enableAutoSIPrefix(False)
        self._plot.showGrid(x=True, y=True, alpha=0.25)

        zero = pg.InfiniteLine(pos=0.0, angle=0,
                               pen=pg.mkPen(QColor(255, 255, 255, 40), width=1))
        self._plot.addItem(zero)

        self._plot.setDownsampling(mode="peak")
        self._plot.setClipToView(True)
        self._plot.enableAutoRange(axis="y", enable=False)
        root.addWidget(self._plot, 1)

        def _curve(color, width=2.0, dash=None, z=0, vb=None):
            pen = pg.mkPen(color=color, width=width)
            if dash:
                pen.setDashPattern(dash)
            if vb is self._vb_right:
                c = pg.PlotDataItem([], [], pen=pen)
                vb.addItem(c)
            else:
                c = self._plot.plot([], [], pen=pen)
            c.setZValue(z)
            return c

        self._c_u = _curve(metric_color("u"), width=1.5, z=1, vb=self._vb_right)
        self._c_pe = _curve(metric_color("pe"), width=1.4, z=2)
        self._c_tp = _curve(metric_color("tp"), width=1.8, dash=[4, 5], z=3)
        self._c_cp = _curve(metric_color("cp"), width=2.2, z=4)

        self._curves = {
            "cp": self._c_cp, "tp": self._c_tp,
            "pe": self._c_pe, "u": self._c_u,
        }

        self._redraw = QTimer(self)
        self._redraw.setInterval(_REDRAW_MS)
        self._redraw.timeout.connect(self._refresh)
        self._redraw.start()

    def add_sample(self, data: dict) -> None:
        if pg is None or self._paused or "cp" not in data:
            return
        if not self._clock.isValid():
            self._clock.start()
        t = self._clock.elapsed() / 1000.0
        row: dict = {"t": t, "cp": data["cp"]}
        for k in ("tp", "pe", "u"):
            if k in data:
                row[k] = data[k]
        self._rows.append(row)
        t_min = t - WINDOW_S
        while self._rows and self._rows[0]["t"] < t_min:
            self._rows.popleft()
        self._dirty = True

    def clear(self) -> None:
        self._rows.clear()
        self._dirty = True

    def _visible_rows(self) -> list[dict]:
        if not self._rows:
            return []
        t_max = self._rows[-1]["t"]
        t_min = t_max - WINDOW_S
        return [r for r in self._rows if r["t"] >= t_min]

    def _refresh(self) -> None:
        if pg is None or not self._dirty:
            return
        self._dirty = False
        visible = self._visible_rows()

        for key, curve in self._curves.items():
            xs, ys = [], []
            for r in visible:
                if key in r:
                    xs.append(r["t"])
                    ys.append(r[key])
            curve.setData(xs, ys)

        has = {k: any(k in r for r in visible) for k in _PLOT_KEYS}
        for key, lbl in self._legend_labels.items():
            col = metric_color(key)
            faint = COLORS["text_faint"]
            lbl.setStyleSheet(
                f"color:{col if has[key] else faint}; font-size:11px; "
                f"{'font-weight:600;' if has[key] else 'opacity:0.45;'} "
                "background:transparent;")

        if visible:
            t_max = visible[-1]["t"]
            t_min = max(visible[0]["t"], t_max - WINDOW_S)
            self._plot.setXRange(t_min, t_max, padding=0.02)

            left_vals: list[float] = []
            for r in visible:
                for k in ("cp", "tp", "pe"):
                    if k in r:
                        left_vals.append(r[k])
            if left_vals:
                ymin, ymax = min(left_vals), max(left_vals)
                span = ymax - ymin or 10.0
                margin = span * _MARGIN_FRAC
                self._plot.setYRange(ymin - margin, ymax + margin)

            u_vals = [r["u"] for r in visible if "u" in r]
            if u_vals:
                umin, umax = min(u_vals), max(u_vals)
                uspan = umax - umin or 0.1
                umargin = uspan * _MARGIN_FRAC
                self._vb_right.setYRange(umin - umargin, umax + umargin)

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
