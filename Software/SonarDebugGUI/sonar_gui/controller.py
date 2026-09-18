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

# Ответы, по которым плата подтверждает удержание вала, не отвечая ok:hold=:
# en и джог сами снимают hold=0 (main.c CMD_ENABLE и CMD_CONTINUOUS), иначе
# индикатор показывал бы «обмотки обесточены» на уже работающем вале.
_HOLD_RESTORED_REPLIES = ("ok:en", "ok:t=+", "ok:t=-")


def _extra_confirmed(line: str) -> tuple[str, int] | None:
    """Подтверждения, которых нет в protocol.parse_ok_reply.

    Возвращает (ключ DeviceState, значение) для ok:om=N и ok:hold=N либо None.
    """
    s = line.strip()
    if s in _HOLD_RESTORED_REPLIES:
        return ("hold", 1)
    for pfx, key in (("ok:om=", "output_mode"), ("ok:hold=", "hold")):
        if s.startswith(pfx):
            try:
                return (key, int(s[len(pfx):]))
            except ValueError:
                return None
    return None


class DeviceClient(QObject):
    telemetry = Signal(dict)         # разобранная телеметрия
    mcfg = Signal(dict)              # ответ mcfg
    sync_status = Signal(dict)       # ответ на запрос `sync` (режим, пины, счётчик)
    reply = Signal(str)              # любая строка-ответ (ok:/err:/mcfg)
    connected = Signal(bool)         # состояние канала
    conn_error = Signal(str)         # ошибка канала (текст пользователю)
    validation_error = Signal(str)   # команда не прошла клиентскую проверку
    response_timeout = Signal(str)   # на команду не пришёл ответ
    scan_sector = Signal(object)     # (start, end) или None — для подсветки на диаграмме
    param_confirmed = Signal(str, object)  # ключ DeviceState, значение из ok:
    param_queried = Signal(str, object)    # то же ответом на запрос `om` / `hold`
    board_rebooted = Signal(dict)    # пришёл boot: — плата поднялась заново

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
        if kind == "sync":
            self._resolve_pending()
            data = P.parse_sync(line)
            if data:
                self.sync_status.emit(data)
            self.reply.emit(line)
            return
        if kind == "reply":
            self._resolve_pending()
            parsed = P.parse_ok_reply(line)
            if parsed:
                self.param_confirmed.emit(*parsed)
            extra = _extra_confirmed(line)
            if extra:
                self.param_confirmed.emit(*extra)
            # Ответ на запрос состояния (om=N / hold=N без префикса ok:) — это
            # не эхо нашей команды, а фактическое состояние платы, поэтому кроме
            # обычного подтверждения идёт отдельным сигналом: виджеты обязаны
            # ему подчиниться, даже если значение не изменилось (см. MainWindow._wire).
            queried = P.parse_state_reply(line)
            if queried:
                self.param_confirmed.emit(*queried)
                self.param_queried.emit(*queried)
            self.reply.emit(line)
            return
        # прочие строки (напр. enc:ok при diag/старте, err:motor_init) — не ответ
        # на команду в смысле FIFO, очередь не трогаем.
        boot = P.parse_boot(line)
        if boot is not None:
            self._on_boot_banner(boot)
        self.reply.emit(line)

    def _on_boot_banner(self, info: dict) -> None:
        """Плата объявила о своём перезапуске (main.c Boot_Banner).

        Всё, что было о ней известно, устарело разом: прошивка поднялась со
        стартовыми значениями (в т.ч. hold=1 — обмотки снова под током), а
        команды, ждавшие ответа, ушли вместе с прошлым запуском. Ожидание
        ответов снимаем здесь, иначе по каждой из них без толку сработает
        таймаут; перезапрос состояния делает MainWindow по board_rebooted.
        """
        if self._pending:
            lost = ", ".join(self._pending)
            self._logger.log_err(
                f"Плата перезапустилась, ответа уже не будет на: {lost}")
        self._cancel_timeout()
        self.board_rebooted.emit(info)

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
