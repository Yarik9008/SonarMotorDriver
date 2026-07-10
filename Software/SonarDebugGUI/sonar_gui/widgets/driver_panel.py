"""DriverPanel — токи (irun/ihold/icur), микрошаг и запрос конфигурации (mcfg)."""
from __future__ import annotations

from typing import Callable

from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QComboBox, QGridLayout, QGroupBox, QHBoxLayout,
                               QLabel, QPushButton, QSpinBox, QVBoxLayout)

from .. import protocol as P
from ..theme import COLORS


def _mono(px: int) -> QFont:
    f = QFont()
    f.setFamilies(["Cascadia Mono", "Consolas", "monospace"])
    f.setPixelSize(px)
    return f


class DriverPanel(QGroupBox):
    def __init__(self, send: Callable[[str], bool], parent=None):
        super().__init__("ДРАЙВЕР TMC2209", parent)
        self._send = send
        d = P.DEFAULTS
        root = QVBoxLayout(self)
        root.setSpacing(6)

        # Токи движения/удержания: спин + компактная кнопка применения
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setColumnStretch(1, 1)
        self._irun = self._spin(d.irun)
        self._ihold = self._spin(d.ihold)

        cap_run = QLabel("I движ., мА")
        cap_run.setProperty("dim", "true")
        grid.addWidget(cap_run, 0, 0)
        grid.addWidget(self._irun, 0, 1)
        b_irun = QPushButton("✓")
        b_irun.setFixedWidth(34)
        b_irun.setToolTip("Применить ток движения (irun)")
        b_irun.clicked.connect(lambda: self._send(P.cmd_irun(self._irun.value())))
        grid.addWidget(b_irun, 0, 2)

        cap_hold = QLabel("I удерж., мА")
        cap_hold.setProperty("dim", "true")
        grid.addWidget(cap_hold, 1, 0)
        grid.addWidget(self._ihold, 1, 1)
        b_ihold = QPushButton("✓")
        b_ihold.setFixedWidth(34)
        b_ihold.setToolTip("Применить ток удержания (ihold)")
        b_ihold.clicked.connect(lambda: self._send(P.cmd_ihold(self._ihold.value())))
        grid.addWidget(b_ihold, 1, 2)
        root.addLayout(grid)

        # Микрошаг
        row_ms = QHBoxLayout()
        cap_ms = QLabel("Микрошаг")
        cap_ms.setProperty("dim", "true")
        row_ms.addWidget(cap_ms)
        self._mstep = QComboBox()
        for v in P.MSTEP_VALUES:
            self._mstep.addItem(str(v), v)
        self._mstep.setCurrentText(str(d.microsteps))
        row_ms.addWidget(self._mstep, 1)
        b_ms = QPushButton("✓")
        b_ms.setFixedWidth(34)
        b_ms.setToolTip("Применить микрошаг (mstep) — только на остановленном моторе")
        b_ms.clicked.connect(lambda: self._send(P.cmd_mstep(self._mstep.currentData())))
        row_ms.addWidget(b_ms)
        root.addLayout(row_ms)

        # Групповые действия
        row_act = QHBoxLayout()
        b_icur = QPushButton("icur: оба")
        b_icur.setToolTip("Применить оба тока одной командой (icur)")
        b_icur.clicked.connect(
            lambda: self._send(P.cmd_icur(self._irun.value(), self._ihold.value())))
        b_cfg = QPushButton("Обновить (mcfg)")
        b_cfg.setToolTip("Запросить текущую конфигурацию драйвера")
        b_cfg.clicked.connect(lambda: self._send(P.cmd_mcfg()))
        # Позволяем кнопкам сжиматься — ширина панели ограничена 400px (§0)
        b_icur.setMinimumWidth(80)
        b_cfg.setMinimumWidth(80)
        row_act.addWidget(b_icur, 1)
        row_act.addWidget(b_cfg, 1)
        root.addLayout(row_act)

        # Строка конфигурации — моно-текст в «колодце»
        self._cfg = QLabel("mode=?  run=?  hold=?  microsteps=?  ready=?")
        self._cfg.setFont(_mono(10))
        self._cfg.setWordWrap(True)
        self._cfg.setStyleSheet(
            f"color: {COLORS['text_dim']};"
            f" background: {COLORS.get('well', '#0b0f16')};"
            f" border: 1px solid {COLORS.get('border', '#1f2735')};"
            " border-radius: 6px; padding: 4px;")
        root.addWidget(self._cfg)

    @staticmethod
    def _spin(val) -> QSpinBox:
        sp = QSpinBox()
        sp.setRange(P.CURRENT_MIN, P.CURRENT_MAX)
        sp.setValue(val)
        return sp

    def update_mcfg(self, d: dict) -> None:
        self._cfg.setText(
            f"mode={d.get('mode', '?')}  run={d.get('run', '?')}  "
            f"hold={d.get('hold', '?')}  microsteps={d.get('microsteps', '?')}  "
            f"ready={d.get('ready', '?')}")
