"""ConnectionPanel — выбор канала (Симулятор / Реальный COM) и подключение."""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QComboBox, QGroupBox, QHBoxLayout, QLabel,
                               QPushButton, QRadioButton, QVBoxLayout)

from ..transport.serial_transport import find_stm32_port, list_serial_ports


def _set_chip(label: QLabel, state: str, text: str | None = None) -> None:
    """Ставит чип-состояние (ok/warn/err/off) и заново применяет стиль."""
    if text is not None:
        label.setText(text)
    label.setProperty("chip", state)
    st = label.style()
    st.unpolish(label)
    st.polish(label)


class ConnectionPanel(QGroupBox):
    connect_requested = Signal(str, str)   # mode ('sim'|'serial'), port
    disconnect_requested = Signal()

    def __init__(self, parent=None):
        super().__init__("ПОДКЛЮЧЕНИЕ", parent)
        self._connected = False

        root = QVBoxLayout(self)
        root.setSpacing(6)

        # Режим канала: симулятор или реальный последовательный порт
        row_mode = QHBoxLayout()
        self._rb_sim = QRadioButton("Симулятор")
        self._rb_serial = QRadioButton("Реальный COM")
        self._rb_sim.setChecked(True)
        self._rb_sim.toggled.connect(self._on_mode)
        row_mode.addWidget(self._rb_sim)
        row_mode.addWidget(self._rb_serial)
        row_mode.addStretch(1)
        root.addLayout(row_mode)

        # Выбор порта: комбо не должно распирать панель по ширине
        row_port = QHBoxLayout()
        lbl_port = QLabel("Порт")
        lbl_port.setProperty("dim", "true")
        self._port = QComboBox()
        self._port.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self._port.setMinimumContentsLength(14)
        self._btn_refresh = QPushButton("⟳")
        self._btn_refresh.setFixedWidth(34)
        self._btn_refresh.setToolTip("Обновить список портов")
        self._btn_refresh.clicked.connect(self.refresh_ports)
        row_port.addWidget(lbl_port)
        row_port.addWidget(self._port, 1)
        row_port.addWidget(self._btn_refresh)
        root.addLayout(row_port)

        self._btn = QPushButton("Подключить")
        self._btn.setObjectName("primaryButton")
        self._btn.clicked.connect(self._on_button)
        root.addWidget(self._btn)

        # Статус подключения — чип (off / ok / err)
        row_status = QHBoxLayout()
        self._status = QLabel()
        _set_chip(self._status, "off", "Не подключено")
        row_status.addWidget(self._status)
        row_status.addStretch(1)
        root.addLayout(row_status)

        self.refresh_ports()
        self._on_mode()

    def refresh_ports(self) -> None:
        cur = self._port.currentText()
        self._port.clear()
        for dev, desc in list_serial_ports():
            self._port.addItem(f"{dev} — {desc}", dev)
        auto = find_stm32_port()
        if auto:
            idx = self._port.findData(auto)
            if idx >= 0:
                self._port.setCurrentIndex(idx)
        elif cur:
            i = self._port.findText(cur)
            if i >= 0:
                self._port.setCurrentIndex(i)

    def _on_mode(self) -> None:
        serial = self._rb_serial.isChecked()
        self._port.setEnabled(serial and not self._connected)
        self._btn_refresh.setEnabled(serial and not self._connected)

    def _on_button(self) -> None:
        if self._connected:
            self.disconnect_requested.emit()
            return
        if self._rb_serial.isChecked():
            port = self._port.currentData() or self._port.currentText().split(" ")[0]
            if not port:
                self.set_status("Выберите порт")
                return
            self.connect_requested.emit("serial", port)
        else:
            self.connect_requested.emit("sim", "")

    def set_connected(self, on: bool) -> None:
        self._connected = on
        self._btn.setText("Отключить" if on else "Подключить")
        self._btn.setObjectName("" if on else "primaryButton")
        self._btn.style().unpolish(self._btn)
        self._btn.style().polish(self._btn)
        self._rb_sim.setEnabled(not on)
        self._rb_serial.setEnabled(not on)
        self._on_mode()

    def set_chip(self, state: str, text: str) -> None:
        """Прямое управление чипом статуса: state in ('ok','warn','err','off')."""
        _set_chip(self._status, state, text)

    def set_status(self, text: str) -> None:
        """Совместимый API: состояние чипа выводится из текста."""
        low = text.lower()
        if "ошибк" in low or "выберите" in low:
            state = "err"
        elif "не подключ" in low or low.startswith("нет "):
            state = "off"
        elif "подключ" in low:
            state = "ok"
        else:
            state = "off"
        _set_chip(self._status, state, text)
