"""ConnectionPanel — выбор канала (Симулятор / Реальный COM) и подключение."""
from __future__ import annotations

from PySide6.QtCore import Signal, Qt
from PySide6.QtWidgets import (QButtonGroup, QComboBox, QGroupBox, QHBoxLayout,
                               QLabel, QPushButton, QRadioButton, QVBoxLayout)

from ..transport.serial_transport import find_stm32_port, list_serial_ports


def _set_chip(label: QLabel, state: str, text: str | None = None) -> None:
    if text is not None:
        label.setText(text)
    label.setProperty("chip", state)
    st = label.style()
    st.unpolish(label)
    st.polish(label)


_PORT_LABEL_MAX = 34  # символов в закрытом комбобоксе; полный текст — в тултипе


def _elide(text: str, max_len: int = _PORT_LABEL_MAX) -> str:
    return text if len(text) <= max_len else text[:max_len - 1].rstrip() + "…"


class ConnectionPanel(QGroupBox):
    connect_requested = Signal(str, str)
    disconnect_requested = Signal()

    def __init__(self, parent=None):
        super().__init__("ПОДКЛЮЧЕНИЕ", parent)
        self._connected = False

        root = QVBoxLayout(self)
        root.setSpacing(6)

        row_mode = QHBoxLayout()
        self._rb_sim = QRadioButton("Симулятор")
        self._rb_serial = QRadioButton("Реальный COM-порт")
        self._rb_sim.setChecked(True)
        self._mode_grp = QButtonGroup(self)
        self._mode_grp.addButton(self._rb_sim)
        self._mode_grp.addButton(self._rb_serial)
        self._rb_sim.toggled.connect(self._on_mode)
        row_mode.addWidget(self._rb_sim)
        row_mode.addWidget(self._rb_serial)
        row_mode.addStretch(1)
        root.addLayout(row_mode)

        row_port = QHBoxLayout()
        lbl_port = QLabel("Порт")
        lbl_port.setProperty("dim", "true")
        self._port = QComboBox()
        self._port.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self._port.setMinimumContentsLength(12)
        self._port.setMinimumWidth(120)
        self._port.currentIndexChanged.connect(self._update_port_tooltip)
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

        row_status = QHBoxLayout()
        self._status = QLabel()
        self._status.setMinimumWidth(160)
        self._status.setTextFormat(Qt.TextFormat.PlainText)
        _set_chip(self._status, "off", "Не подключено")
        row_status.addWidget(self._status, 1)
        root.addLayout(row_status)

        self.refresh_ports()
        self._on_mode()

    def refresh_ports(self) -> None:
        cur = self._port.currentData()
        self._port.clear()
        for dev, desc in list_serial_ports():
            full = f"{dev} — {desc}"
            self._port.addItem(_elide(full), dev)
            idx = self._port.count() - 1
            self._port.setItemData(idx, full, Qt.ItemDataRole.ToolTipRole)
        auto = find_stm32_port()
        if auto:
            idx = self._port.findData(auto)
            if idx >= 0:
                self._port.setCurrentIndex(idx)
        elif cur:
            idx = self._port.findData(cur)
            if idx >= 0:
                self._port.setCurrentIndex(idx)
        self._update_port_tooltip()

    def _update_port_tooltip(self) -> None:
        tip = self._port.itemData(self._port.currentIndex(), Qt.ItemDataRole.ToolTipRole)
        self._port.setToolTip(tip or "")

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

    def set_status(self, text: str) -> None:
        low = text.lower()
        if "ошибк" in low or "выберите" in low:
            state = "err"
        elif "не подключ" in low or low.startswith("нет "):
            state = "off"
        elif "подключ" in low or "симулятор" in low:
            state = "ok"
        else:
            state = "off"
        self._status.setText(text)
        self._status.setToolTip(text)
        _set_chip(self._status, state)

    def set_enabled_controls(self, on: bool) -> None:
        if not self._connected:
            return
        self._btn.setEnabled(on)
