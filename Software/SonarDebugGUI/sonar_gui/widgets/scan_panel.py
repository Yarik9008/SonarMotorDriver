"""ScanPanel — сектор (зигзаг) и бесконечный скан. Сообщает сектор для диаграммы.

Здесь же — режим синхронизации скана (sync=0/1/2): выбор источника перехода
к следующей точке и индикатор состояния (подтверждённый режим, уровни
SYNC_IN/SYNC_OUT, счётчик фронтов из запроса `sync`).
"""
from __future__ import annotations

from typing import Callable

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QPushButton, QSpinBox,
                               QVBoxLayout)

from .. import protocol as P
from ..theme import COLORS


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
        # Границы сектора живут в кольце [0,360): end отсчитывается от start в
        # сторону возрастания угла, поэтому end < start — это сектор через ноль
        # (350 → 10 = 20°), а не ошибка ввода.
        self._start = self._dspin(0.0, 0.0, 360.0, wrap=True)
        self._end = self._dspin(180.0, 0.0, 360.0, wrap=True)
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

        # Синхронизация: источник перехода к следующей точке (sync=0/1/2)
        sync_row = QHBoxLayout()
        cap = QLabel("Синхронизация")
        cap.setProperty("dim", "true")
        sync_row.addWidget(cap)
        self._sync_confirmed: int | None = None   # последний подтверждённый sync=
        self._sync_box = QComboBox()
        for mode in P.SYNC_MODE_VALUES:
            self._sync_box.addItem(P.SYNC_MODE_LABELS[mode], mode)
        self._sync_box.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self._sync_box.setMinimumContentsLength(14)
        # activated — только действие пользователя (программный setCurrentIndex
        # из apply_state сигнал не даёт, петли подтверждения не возникает)
        self._sync_box.activated.connect(self._on_sync_mode)
        sync_row.addWidget(self._sync_box, 1)
        self._sync_refresh = QPushButton("⟳")
        self._sync_refresh.setToolTip("Запросить состояние синхронизации (sync)")
        self._sync_refresh.setFixedWidth(28)
        self._sync_refresh.clicked.connect(
            lambda: self._send(P.cmd_sync_query()))
        sync_row.addWidget(self._sync_refresh)
        root.addLayout(sync_row)

        self._sync_status = QLabel()
        self._sync_status.setProperty("dim", "true")
        root.addWidget(self._sync_status)
        self._show_sync_status(None)

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
    def _dspin(val, lo, hi, wrap: bool = False) -> QDoubleSpinBox:
        sp = QDoubleSpinBox()
        sp.setRange(lo, hi)
        sp.setWrapping(wrap)
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

    # ── Синхронизация ──────────────────────────────────────────────────────
    def _on_sync_mode(self) -> None:
        self._send(P.cmd_sync_mode(self._sync_box.currentData()))

    def _show_sync_status(self, st) -> None:
        def lvl(v):
            return "–" if v is None else str(v)

        mode = getattr(st, "sync_mode", None) if st is not None else None
        label = P.SYNC_MODE_LABELS.get(mode, "—")
        edges = getattr(st, "sync_edges", None) if st is not None else None
        self._sync_status.setText(
            f"режим: {label} · SYNC_IN={lvl(getattr(st, 'sync_in', None) if st else None)}"
            f" SYNC_OUT={lvl(getattr(st, 'sync_out', None) if st else None)}"
            f" · фронтов: {lvl(edges)}")
        self._mark_sync_mismatch(mode)

    def _mark_sync_mismatch(self, mode: int | None) -> None:
        """Подсвечивает расхождение подтверждённого платой режима с выбором
        в списке — как эхо om= в ConnectionPanel и hold= в MotorPanel:
        пока ответ ok:sync= в пути (или команда вовсе не дошла),
        оператор видит, что на плате ещё прежний источник перехода."""
        if mode is None:
            self._sync_status.setStyleSheet("")
        elif mode != self._sync_box.currentData():
            self._sync_status.setStyleSheet(f"color: {COLORS['mismatch']};")
        else:
            self._sync_status.setStyleSheet(f"color: {COLORS['text_dim']};")

    def apply_state(self, st) -> None:
        """Отражает DeviceState: подтверждённый режим и статус пинов."""
        mode = getattr(st, "sync_mode", None)
        # Список подтягиваем только на смену подтверждённого значения (см.
        # ConnectionPanel.apply_state): apply_state зовётся на каждый кадр
        # телеметрии (при op=4 — сотни раз в секунду), и возврат к прежнему
        # значению перебивал бы свежий выбор пользователя, пока ответ
        # ok:sync= ещё в пути.
        if mode != self._sync_confirmed:
            self._sync_confirmed = mode
            if mode is not None:
                i = self._sync_box.findData(mode)
                if i >= 0:
                    self._sync_box.setCurrentIndex(i)
        self._show_sync_status(st)

    def apply_sync_reply(self, data: dict) -> None:
        """Ответ на явный запрос `sync` (кнопка ⟳, синхронизация при
        подключении) — авторитетен: список становится тем, что на плате.

        Охранник apply_state бережёт свежий выбор пользователя от потока
        телеметрии, но ответ на запрос состояния глушить не должен: если
        команда sync= до платы не дошла, плата ответит прежним режимом,
        и по одному лишь «подтверждение изменилось» список навсегда остался бы
        на несбывшемся выборе — а это точки скана по внешнему триггеру,
        которого на самом деле нет. См. MainWindow._wire.
        """
        mode = data.get("sync_mode")
        if mode is None:
            return
        self._sync_confirmed = mode
        i = self._sync_box.findData(mode)
        if i >= 0:
            # Программная установка не даёт activated — команда sync= обратно
            # не уходит (см. подключение _sync_box.activated).
            self._sync_box.setCurrentIndex(i)
        self._mark_sync_mismatch(mode)

    def reset(self) -> None:
        self._sync_confirmed = None
        self._show_sync_status(None)

    def set_enabled_controls(self, on: bool) -> None:
        self._mode.setEnabled(on)
        self._start.setEnabled(on)
        self._end.setEnabled(on)
        self._step.setEnabled(on)
        self._delay.setEnabled(on)
        self._sync_box.setEnabled(on)
        for btn in self.findChildren(QPushButton):
            btn.setEnabled(on)
