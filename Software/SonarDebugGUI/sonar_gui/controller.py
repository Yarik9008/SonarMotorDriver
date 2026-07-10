"""DeviceClient — связка транспорта и UI.

- отправка команд с клиентской валидацией (обход «молчания» прошивки);
- таймаут ответа: если на команду не пришло ok:/err:, предупреждаем;
- разбор входящих строк и раздача: телеметрия / mcfg / ответ.
"""
from __future__ import annotations

from collections import deque

from PySide6.QtCore import QObject, QTimer, Signal

from . import protocol as P
from .logger import Logger
from .transport.base import Transport

RESPONSE_TIMEOUT_MS = 400


class DeviceClient(QObject):
    telemetry = Signal(dict)         # разобранная телеметрия
    mcfg = Signal(dict)              # ответ mcfg
    reply = Signal(str)              # любая строка-ответ (ok:/err:/mcfg)
    connected = Signal(bool)         # состояние канала
    conn_error = Signal(str)         # ошибка канала (текст пользователю)
    validation_error = Signal(str)   # команда не прошла клиентскую проверку
    response_timeout = Signal(str)   # на команду не пришёл ответ
    scan_sector = Signal(object)     # (start, end) или None — для подсветки на диаграмме
    param_confirmed = Signal(str, object)  # ключ DeviceState, значение из ok:

    def __init__(self, logger: Logger):
        super().__init__()
        self._logger = logger
        self._transport: Transport | None = None
        # FIFO ожидающих ответа команд: прошивка отвечает по одной строке
        # ok:/err:/mode= на каждую команду в порядке отправки. Очередь (а не
        # одно поле) корректно отслеживает пачку команд, отправленных подряд.
        self._pending: deque[str] = deque()
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(RESPONSE_TIMEOUT_MS)
        self._timer.timeout.connect(self._on_timeout)

    # ── Подключение ────────────────────────────────────────────────────────
    def connect_transport(self, transport: Transport) -> None:
        self.disconnect()
        self._transport = transport
        transport.line_received.connect(self._on_line)
        transport.opened.connect(self._on_opened)
        transport.closed.connect(self._on_closed)
        transport.error.connect(self._on_error)
        self._logger.log_info(f"Подключение: {transport.describe()}")
        transport.open()

    def disconnect(self) -> None:
        self._cancel_timeout()
        if self._transport is None:
            return
        t, self._transport = self._transport, None
        try:
            t.line_received.disconnect(self._on_line)
            t.opened.disconnect(self._on_opened)
            t.closed.disconnect(self._on_closed)
            t.error.disconnect(self._on_error)
        except (RuntimeError, TypeError):
            pass
        t.close()
        # Явное отключение: сигнал closed уже отвязан от _on_closed (чтобы не
        # ловить события закрываемого транспорта), поэтому UI об отключении
        # уведомляем напрямую — иначе состояние «подключено» зависало бы.
        self.connected.emit(False)

    @property
    def is_connected(self) -> bool:
        return self._transport is not None and self._transport.is_open

    # ── Отправка ───────────────────────────────────────────────────────────
    def send(self, cmd: str, validate: bool = True) -> bool:
        cmd = cmd.strip()
        if not cmd:
            return False
        if self._transport is None or not self._transport.is_open:
            self.validation_error.emit("Нет подключения")
            return False
        if validate:
            ok, reason = P.validate(cmd)
            if not ok:
                self._logger.log_err(f"{cmd} — {reason}")
                self.validation_error.emit(reason)
                return False
        self._transport.write_line(cmd)
        self._logger.log_tx(cmd)
        if P.expects_reply(cmd):
            self._pending.append(cmd)
            if not self._timer.isActive():
                self._timer.start()          # таймаут отсчитывается для головы очереди
        return True

    # ── Приём ──────────────────────────────────────────────────────────────
    def _on_line(self, line: str) -> None:
        self._logger.log_rx(line)
        kind = P.classify_line(line)
        if kind == "telemetry":
            data = P.parse_telemetry(line)
            if data:
                self.telemetry.emit(data)
            return
        if kind == "mcfg":
            self._resolve_pending()
            data = P.parse_mcfg(line)
            if data:
                self.mcfg.emit(data)
            self.reply.emit(line)
            return
        if kind == "reply":
            self._resolve_pending()
            parsed = P.parse_ok_reply(line)
            if parsed:
                self.param_confirmed.emit(*parsed)
            self.reply.emit(line)
            return
        # прочие строки (напр. enc:ok при diag/старте, err:motor_init) — не ответ
        # на команду в смысле FIFO, очередь не трогаем.
        self.reply.emit(line)

    def _resolve_pending(self) -> None:
        """Пришёл ответ на голову очереди: снимаем её и перевзводим таймаут."""
        if self._pending:
            self._pending.popleft()
        self._timer.stop()
        if self._pending:
            self._timer.start()

    def _on_timeout(self) -> None:
        if self._pending:
            cmd = self._pending.popleft()
            self._logger.log_err(f"Нет ответа на: {cmd}")
            self.response_timeout.emit(cmd)
        if self._pending:
            self._timer.start()              # ждём ответ на следующую команду

    def _cancel_timeout(self) -> None:
        self._pending.clear()
        self._timer.stop()

    def _on_opened(self) -> None:
        self._logger.log_info("Канал открыт")
        self.connected.emit(True)

    def _on_closed(self) -> None:
        self._cancel_timeout()
        self._logger.log_info("Канал закрыт")
        self.connected.emit(False)

    def _on_error(self, msg: str) -> None:
        self._logger.log_err(msg)
        self.conn_error.emit(msg)

    # ── Сектор скана (клиентское знание — для диаграммы) ────────────────────
    def set_scan_sector(self, sector) -> None:
        self.scan_sector.emit(sector)
