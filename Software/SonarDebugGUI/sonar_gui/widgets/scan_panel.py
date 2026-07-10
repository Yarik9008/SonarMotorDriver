"""ScanPanel — сектор (зигзаг) и бесконечный скан. Сообщает сектор для диаграммы."""
from __future__ import annotations

from typing import Callable

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QPushButton, QSpinBox,
                               QVBoxLayout)

from .. import protocol as P


class ScanPanel(QGroupBox):
    sector_changed = Signal(object)      # (start, end) или None

    def __init__(self, send: Callable[[str], bool], parent=None):
        super().__init__("СКАНИРОВАНИЕ", parent)
        self._send = send
        root = QVBoxLayout(self)
        root.setSpacing(6)

        self._mode = QComboBox()
        self._mode.addItem("Зигзаг (сектор)", "sector")
        self._mode.addItem("Бесконечный  +", "+")
        self._mode.addItem("Бесконечный  -", "-")
        self._mode.currentIndexChanged.connect(self._on_mode)
        # Не даём комбо распирать панель шире 400px (§0)
        self._mode.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self._mode.setMinimumContentsLength(14)
        self._mode.setMinimumWidth(90)
        root.addWidget(self._mode)

        # Параметры сеткой 2×2 (пары «подпись + спинбокс»)
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        self._start = self._dspin(0.0, -1_000_000, 1_000_000)
        self._end = self._dspin(180.0, -1_000_000, 1_000_000)
        self._step = self._dspin(10.0, 0.01, 100_000)
        self._delay = QSpinBox()
        self._delay.setRange(1, 60_000)
        self._delay.setValue(100)
        self._delay.setMinimumWidth(72)
        for r, pairs in enumerate((
            (("Старт, °", self._start), ("Конец, °", self._end)),
            (("Шаг, °", self._step), ("Пауза, мс", self._delay)),
        )):
            for col, (caption, widget) in enumerate(pairs):
                cap = QLabel(caption)
                cap.setProperty("dim", "true")
                grid.addWidget(cap, r, col * 2)
                grid.addWidget(widget, r, col * 2 + 1)
        root.addLayout(grid)

        row = QHBoxLayout()
        b_start = QPushButton("▶ Старт")
        b_start.setObjectName("primaryButton")
        b_start.clicked.connect(self._start_scan)
        b_stop = QPushButton("■ Стоп")
        b_stop.clicked.connect(self._stop_scan)
        row.addWidget(b_start, 1)
        row.addWidget(b_stop, 1)
        root.addLayout(row)

    @staticmethod
    def _dspin(val, lo, hi) -> QDoubleSpinBox:
        sp = QDoubleSpinBox()
        sp.setRange(lo, hi)
        sp.setDecimals(2)
        sp.setValue(val)
        # Явный минимум: min-подсказка спинбокса с диапазоном ±1e6
        # иначе распирает панель шире 400px (§0)
        sp.setMinimumWidth(72)
        return sp

    def _on_mode(self) -> None:
        self._end.setEnabled(self._mode.currentData() == "sector")

    def _start_scan(self) -> None:
        mode = self._mode.currentData()
        s = self._start.value()
        step = self._step.value()
        delay = self._delay.value()
        if mode == "sector":
            e = self._end.value()
            cmd = P.cmd_scan_sector(s, e, step, delay)
            if self._send(cmd):
                self.sector_changed.emit((s, e))
        else:
            cmd = P.cmd_scan_infinite(s, mode, step, delay)
            if self._send(cmd):
                self.sector_changed.emit(None)

    def _stop_scan(self) -> None:
        self._send(P.cmd_stop())
        self.sector_changed.emit(None)
