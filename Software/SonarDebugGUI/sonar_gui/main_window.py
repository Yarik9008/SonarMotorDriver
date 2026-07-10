"""MainWindow — сборка отладчика: шапка, панели, PPI-диаграмма, графики, консоль."""
from __future__ import annotations

from PySide6.QtCore import QLocale, QSettings, QTimer, Qt
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (QAbstractSpinBox, QFrame, QMainWindow,
                               QScrollArea, QSplitter, QTabWidget, QVBoxLayout,
                               QWidget)

from . import protocol as P
from .controller import DeviceClient
from .device_state import DeviceStateModel
from .logger import Logger
from .transport import SerialTransport, SimTransport
from .widgets.collapsible_section import CollapsibleSection
from .widgets.connection_panel import ConnectionPanel
from .widgets.console_panel import ConsolePanel
from .widgets.header_bar import HeaderBar
from .widgets.motor_panel import MotorPanel
from .widgets.pid_panel import PidPanel
from .widgets.ppi_dial import PPIDialWidget
from .widgets.scan_panel import ScanPanel
from .widgets.telemetry_plot import TelemetryPlot

_LEFT_COLUMN_W = 460      # начальная и минимальная ширина левой колонки
_SYNC_DELAY_MS = 150


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SonarDebugGUI — отладчик FW_SonarMotorDriver")
        self.resize(1180, 740)

        self._settings = QSettings("SonarMotorDriver", "SonarDebugGUI")
        self.logger = Logger()
        self.client = DeviceClient(self.logger)
        self.state = DeviceStateModel(self)
        self._transport = None

        self.header = HeaderBar()
        self.conn = ConnectionPanel(self.client.send)
        self.motor = MotorPanel(self.client.send)
        self.pid = PidPanel(self.client.send)
        self.scan = ScanPanel(self.client.send)
        self.dial = PPIDialWidget()
        self.plot = TelemetryPlot()
        self.console = ConsolePanel(lambda c: self.client.send(c, validate=False))

        self._build_layout()
        self._wire()
        self._setup_shortcuts()

        c_locale = QLocale.c()
        for sb in self.findChildren(QAbstractSpinBox):
            sb.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
            sb.setLocale(c_locale)

        self.dial.set_config(*self.motor.antenna_values())
        self._on_state_changed(self.state.state)
        self.statusBar().showMessage("Готово. Выберите канал и нажмите «Подключить».")

    def _build_layout(self) -> None:
        controls = QWidget()
        cl = QVBoxLayout(controls)
        cl.setContentsMargins(10, 8, 10, 8)
        cl.setSpacing(4)

        cl.addWidget(self.conn)

        sections = [
            ("Мотор", "sect_motor", self.motor),
            ("Регулятор PID", "sect_pid", self.pid),
            ("Сканирование", "sect_scan", self.scan),
        ]
        for title, key, widget in sections:
            sec = CollapsibleSection(title, key, self._settings)
            if hasattr(widget, "setTitle"):
                widget.setTitle("")
                widget.setFlat(True)
            sec.content_layout().addWidget(widget)
            cl.addWidget(sec)

        cl.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(controls)
        scroll.setFrameShape(QFrame.NoFrame)
        # Минимум, а не фикс: колонка тянется вместе с окном/сплиттером,
        # но не сжимается настолько, чтобы обрезать содержимое панелей.
        scroll.setMinimumWidth(_LEFT_COLUMN_W)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        right = QTabWidget()
        right.addTab(self.dial, "Диаграмма")
        right.addTab(self.plot, "Графики")
        right.addTab(self.console, "Консоль")

        center = QSplitter(Qt.Horizontal)
        center.addWidget(scroll)
        center.addWidget(right)
        center.setStretchFactor(0, 0)
        center.setStretchFactor(1, 1)
        center.setChildrenCollapsible(False)
        center.setSizes([_LEFT_COLUMN_W, 1])

        central = QWidget()
        v = QVBoxLayout(central)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        v.addWidget(self.header)
        v.addWidget(center, 1)
        self.setCentralWidget(central)

    def _setup_shortcuts(self) -> None:
        for key in (Qt.Key.Key_Escape, Qt.Key.Key_Space):
            sc = QShortcut(QKeySequence(key), self)
            sc.setContext(Qt.ShortcutContext.ApplicationShortcut)
            sc.activated.connect(lambda: self.client.send(P.cmd_stop()))

    def _wire(self) -> None:
        self.conn.connect_requested.connect(self._on_connect)
        self.conn.disconnect_requested.connect(self.client.disconnect)

        self.client.connected.connect(self._on_connected)
        self.client.conn_error.connect(self._on_conn_error)
        self.client.telemetry.connect(self.state.apply_telemetry)
        self.client.telemetry.connect(self.plot.add_sample)
        self.client.mcfg.connect(self.motor.update_mcfg)
        self.client.validation_error.connect(
            lambda m: self.statusBar().showMessage(f"⚠ {m}", 4000))
        self.client.response_timeout.connect(
            lambda c: self.statusBar().showMessage(f"⚠ Нет ответа на «{c}»", 4000))
        self.client.scan_sector.connect(self.dial.set_scan)

        self.state.state_changed.connect(self._on_state_changed)
        self.client.param_confirmed.connect(
            lambda k, v: self.state.set_confirmed(k, v))
        self.scan.sector_changed.connect(self.client.set_scan_sector)
        self.motor.antenna_config_changed.connect(self.dial.set_config)
        self.logger.message.connect(self.console.append_log)

    def _on_state_changed(self, st) -> None:
        self.header.apply_state(st)
        self.dial.apply_state(st)
        self.conn.apply_state(st)
        self.motor.apply_state(st)
        self.pid.apply_state(st)

    def _reset_readouts(self, clear_plot: bool) -> None:
        self.state.reset(keep_connection=self.client.is_connected)
        self.conn.reset()
        self.motor.reset()
        self.pid.reset()
        self.dial.set_scan(None)
        if clear_plot:
            self.plot.clear()

    def _sync_device(self) -> None:
        if self.client.is_connected:
            self.client.send(P.cmd_mcfg())

    def _on_connect(self, mode: str, port: str) -> None:
        self._reset_readouts(clear_plot=True)
        if mode == "serial":
            self._transport = SerialTransport(port)
            channel = (port or "SERIAL").upper()
            warn = False
        else:
            self._transport = SimTransport()
            channel = "СИМУЛЯТОР"
            warn = True
        self.state.set_connection(False, channel, warn=warn)
        self.client.connect_transport(self._transport)

    def _on_connected(self, on: bool) -> None:
        self.conn.set_connected(on)
        if on:
            desc = self._transport.describe() if self._transport else ""
            channel = desc
            warn = "симулятор" in desc.lower()
            self.state.set_connection(True, channel, warn=warn)
            txt = f"Подключено: {desc}"
            self.conn.set_status(txt)
            self.statusBar().showMessage(txt, 4000)
            QTimer.singleShot(_SYNC_DELAY_MS, self._sync_device)
            self._set_controls_enabled(True)
        else:
            self._reset_readouts(clear_plot=False)
            self.state.set_connection(False, "НЕ ПОДКЛЮЧЕНО")
            self.conn.set_status("Не подключено")
            self.statusBar().showMessage("Отключено", 4000)
            self._set_controls_enabled(False)

    def _on_conn_error(self, msg: str) -> None:
        self.conn.set_connected(False)
        self._reset_readouts(clear_plot=False)
        self.state.set_connection(False, "ОШИБКА")
        self.conn.set_status("Ошибка подключения")
        self.statusBar().showMessage(f"⚠ {msg.splitlines()[0]}", 8000)
        self._set_controls_enabled(False)

    def _set_controls_enabled(self, on: bool) -> None:
        self.motor.set_enabled_controls(on)
        self.conn.set_enabled_controls(on)
        self.pid.set_enabled_controls(on)
        self.scan.set_enabled_controls(on)
        self.console.set_enabled_controls(on)

    def closeEvent(self, ev) -> None:
        try:
            self.client.disconnect()
            self.logger.close()
        finally:
            super().closeEvent(ev)
