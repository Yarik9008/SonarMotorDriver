"""MainWindow — сборка отладчика: шапка, панели, PPI-диаграмма, графики, консоль."""
from __future__ import annotations

from PySide6.QtCore import QLocale, Qt
from PySide6.QtWidgets import (QAbstractSpinBox, QFrame, QMainWindow,
                               QScrollArea, QSplitter, QVBoxLayout, QWidget)

from .controller import DeviceClient
from .logger import Logger
from .transport import SerialTransport, SimTransport
from .widgets.antenna_config_panel import AntennaConfigPanel
from .widgets.connection_panel import ConnectionPanel
from .widgets.console_panel import ConsolePanel
from .widgets.driver_panel import DriverPanel
from .widgets.header_bar import HeaderBar
from .widgets.motor_panel import MotorPanel
from .widgets.pid_panel import PidPanel
from .widgets.ppi_dial import PPIDialWidget
from .widgets.scan_panel import ScanPanel
from .widgets.telemetry_panel import TelemetryPanel
from .widgets.telemetry_plot import TelemetryPlot

# Ширина левой колонки: панели рассчитаны на ≤400px содержимого,
# горизонтальный скролл запрещён — ничего не режется.
_LEFT_COLUMN_W = 430


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SonarDebugGUI — отладчик FW_SonarMotorDriver")
        self.resize(1240, 800)

        self.logger = Logger()
        self.client = DeviceClient(self.logger)
        self._transport = None
        self._st = {"cp": 0.0, "tp": 0.0, "m": "cl", "ec": 0}
        self._hdr_name = "НЕ ПОДКЛЮЧЕНО"  # короткое имя канала для чипа в шапке

        # ── Виджеты ────────────────────────────────────────────────────────
        self.header = HeaderBar()
        self.conn = ConnectionPanel()
        self.motor = MotorPanel(self.client.send)
        self.telem = TelemetryPanel(self.client.send)
        self.pid = PidPanel(self.client.send)
        self.scan = ScanPanel(self.client.send)
        self.driver = DriverPanel(self.client.send)
        self.antcfg = AntennaConfigPanel()
        self.dial = PPIDialWidget()
        self.plot = TelemetryPlot()
        self.console = ConsolePanel(lambda c: self.client.send(c, validate=False))

        self._build_layout()
        self._wire()

        # Все спин-боксы: без кнопок-«ступенек» (значения вводятся с клавиатуры,
        # применяются кнопкой «✓») и с локалью C — десятичная ТОЧКА, как в
        # протоколе устройства и моноширинных readout'ах (не системная запятая)
        c_locale = QLocale.c()
        for sb in self.findChildren(QAbstractSpinBox):
            sb.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
            sb.setLocale(c_locale)

        self.dial.set_config(*self.antcfg.values())
        self.statusBar().showMessage("Готово. Выберите канал и нажмите «Подключить».")

    # ── Компоновка ─────────────────────────────────────────────────────────
    def _build_layout(self) -> None:
        controls = QWidget()
        cl = QVBoxLayout(controls)
        # Компактная вертикаль: PID влезает на первый экран целиком,
        # шапка «СКАНИРОВАНИЕ» видна как аффорданс прокрутки
        cl.setContentsMargins(10, 8, 10, 8)
        cl.setSpacing(6)
        for w in (self.conn, self.motor, self.telem, self.pid,
                  self.scan, self.driver, self.antcfg):
            cl.addWidget(w)
        cl.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(controls)
        scroll.setFrameShape(QFrame.NoFrame)
        # Фикс ширины + запрет горизонтального скролла: панели не обрезаются
        scroll.setFixedWidth(_LEFT_COLUMN_W)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Правая колонка: PPI-диаграмма (заголовок — в её HUD), график, консоль
        right = QSplitter(Qt.Vertical)
        right.addWidget(self.dial)
        right.addWidget(self.plot)
        right.addWidget(self.console)
        right.setStretchFactor(0, 5)
        right.setStretchFactor(1, 3)
        right.setStretchFactor(2, 2)
        right.setSizes([430, 300, 170])
        right.setChildrenCollapsible(False)

        center = QSplitter(Qt.Horizontal)
        center.addWidget(scroll)
        center.addWidget(right)
        center.setStretchFactor(0, 0)
        center.setStretchFactor(1, 1)
        center.setChildrenCollapsible(False)

        central = QWidget()
        v = QVBoxLayout(central)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        v.addWidget(self.header)
        v.addWidget(center, 1)
        self.setCentralWidget(central)

    # ── Связи ──────────────────────────────────────────────────────────────
    def _wire(self) -> None:
        self.conn.connect_requested.connect(self._on_connect)
        self.conn.disconnect_requested.connect(self.client.disconnect)

        self.client.connected.connect(self._on_connected)
        self.client.conn_error.connect(self._on_conn_error)
        self.client.telemetry.connect(self._on_telemetry)
        self.client.mcfg.connect(self.driver.update_mcfg)
        self.client.validation_error.connect(
            lambda m: self.statusBar().showMessage(f"⚠ {m}", 4000))
        self.client.response_timeout.connect(
            lambda c: self.statusBar().showMessage(f"⚠ Нет ответа на «{c}»", 4000))
        self.client.scan_sector.connect(self.dial.set_scan)

        self.scan.sector_changed.connect(self.client.set_scan_sector)
        self.antcfg.config_changed.connect(self.dial.set_config)
        self.logger.message.connect(self.console.append_log)

    # ── Обработчики ────────────────────────────────────────────────────────
    def _on_connect(self, mode: str, port: str) -> None:
        if mode == "serial":
            self._transport = SerialTransport(port)
            self._hdr_name = (port or "SERIAL").upper()
        else:
            self._transport = SimTransport()
            self._hdr_name = "СИМУЛЯТОР"
        self.client.connect_transport(self._transport)

    def _on_connected(self, on: bool) -> None:
        self.conn.set_connected(on)
        self.dial.set_connected(on)
        if on:
            desc = self._transport.describe() if self._transport else ""
            # Один и тот же текст в чипе панели и статус-баре (единый регистр)
            txt = f"Подключено: {desc}"
            self.conn.set_status(txt)
            # Симулятор — янтарный чип-предупреждение: зелёный только для железа
            self.header.set_connection(True, self._hdr_name,
                                       warn=(self._hdr_name == "СИМУЛЯТОР"))
            self.statusBar().showMessage(txt, 4000)
        else:
            self.conn.set_status("Не подключено")
            self.header.set_connection(False, "НЕ ПОДКЛЮЧЕНО")
            self.header.set_angle(None)
            self.header.set_mode(None)
            self.telem.reset()
            self.dial.set_scan(None)
            self.statusBar().showMessage("Отключено", 4000)

    def _on_conn_error(self, msg: str) -> None:
        self.conn.set_connected(False)
        self.dial.set_connected(False)
        self.conn.set_status("Ошибка подключения")
        self.header.set_connection(False, "ОШИБКА")
        self.header.set_angle(None)
        self.header.set_mode(None)
        self.statusBar().showMessage(f"⚠ {msg.splitlines()[0]}", 8000)

    def _on_telemetry(self, d: dict) -> None:
        for k_src, k_dst in (("cp", "cp"), ("tp", "tp"), ("m", "m"), ("ec", "ec")):
            if k_src in d:
                self._st[k_dst] = d[k_src]
        self.dial.set_state(self._st["cp"], self._st["tp"], self._st["m"], self._st["ec"])
        self.header.set_angle(self.dial.antenna_angle(self._st["cp"]))
        self.header.set_mode(str(self._st["m"]))
        self.plot.add_sample(d)
        self.telem.update_from_telemetry(d)
        self.pid.update_from_telemetry(d)
        self.motor.update_from_telemetry(d)

    # ── Завершение ─────────────────────────────────────────────────────────
    def closeEvent(self, ev) -> None:
        try:
            self.client.disconnect()
            self.logger.close()
        finally:
            super().closeEvent(ev)
