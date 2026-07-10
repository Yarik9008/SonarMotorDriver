"""AntennaConfigPanel — настройки диаграммы: масштаб, смещение нуля, направление.

Прошивка не знает про редуктор/монтаж антенны, поэтому пересчёт
угол = sign*(cp*scale) + offset задаётся здесь. Сохраняется в QSettings.
"""
from __future__ import annotations

from PySide6.QtCore import QSettings, Signal
from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox,
                               QLabel)


class AntennaConfigPanel(QGroupBox):
    config_changed = Signal(float, float, int)   # scale, offset, sign

    def __init__(self, parent=None):
        super().__init__("ДИАГРАММА АНТЕННЫ", parent)
        self._s = QSettings("SonarMotorDriver", "SonarDebugGUI")
        grid = QGridLayout(self)
        grid.setHorizontalSpacing(8)
        grid.setColumnStretch(1, 1)

        cap_scale = QLabel("Масштаб (антенна/вал)")
        cap_scale.setProperty("dim", "true")
        grid.addWidget(cap_scale, 0, 0)
        self._scale = QDoubleSpinBox()
        self._scale.setRange(0.0001, 10000.0)
        self._scale.setDecimals(4)
        self._scale.setValue(float(self._s.value("scale", 1.0)))
        self._scale.setMinimumWidth(90)   # не распирать панель шире 400px (§0)
        grid.addWidget(self._scale, 0, 1)

        cap_offset = QLabel("Смещение нуля, °")
        cap_offset.setProperty("dim", "true")
        grid.addWidget(cap_offset, 1, 0)
        self._offset = QDoubleSpinBox()
        self._offset.setRange(-360.0, 360.0)
        self._offset.setDecimals(2)
        self._offset.setValue(float(self._s.value("offset", 0.0)))
        self._offset.setMinimumWidth(90)
        grid.addWidget(self._offset, 1, 1)

        cap_sign = QLabel("Направление")
        cap_sign.setProperty("dim", "true")
        grid.addWidget(cap_sign, 2, 0)
        self._sign = QComboBox()
        self._sign.addItem("+1 (по часовой)", 1)
        self._sign.addItem("-1 (против)", -1)
        idx = self._sign.findData(int(self._s.value("sign", 1)))
        self._sign.setCurrentIndex(max(0, idx))
        self._sign.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self._sign.setMinimumContentsLength(12)
        self._sign.setMinimumWidth(90)
        grid.addWidget(self._sign, 2, 1)

        self._scale.valueChanged.connect(self._emit)
        self._offset.valueChanged.connect(self._emit)
        self._sign.currentIndexChanged.connect(self._emit)

    def values(self) -> tuple[float, float, int]:
        return self._scale.value(), self._offset.value(), int(self._sign.currentData())

    def _emit(self) -> None:
        scale, offset, sign = self.values()
        self._s.setValue("scale", scale)
        self._s.setValue("offset", offset)
        self._s.setValue("sign", sign)
        self.config_changed.emit(scale, offset, sign)
