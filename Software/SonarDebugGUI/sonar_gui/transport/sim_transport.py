"""SimTransport — встроенный симулятор прошивки в виде транспорта.

Оборачивает FirmwareSimulator: команды идут в модель, ответы и поток телеметрии
эмитятся сигналом line_received — как будто отвечает настоящая плата.
Работает полностью без железа.
"""
from __future__ import annotations

from collections import deque

from PySide6.QtCore import QElapsedTimer, QTimer

from .base import Transport
from ..simulator import FirmwareSimulator

_TICK_MS = 10                   # шаг модели
_TELE_MIN_DEBUG_MS = 20         # минимальный период телеметрии при debug=1 (как в прошивке)


class SimTransport(Transport):
    def __init__(self):
        super().__init__()
        self.sim = FirmwareSimulator()
        self._open = False
        self._tele_accum = 0.0
        self._clock = QElapsedTimer()
        self._timer = QTimer(self)
        self._timer.setInterval(_TICK_MS)
        self._timer.timeout.connect(self._on_tick)
        # Единая исходящая FIFO-очередь (ответы + телеметрия). Всё, что «шлёт
        # плата», проходит через неё и выдаётся строго в порядке добавления —
        # ответ на команду не может обогнать/отстать от телеметрии.
        self._outbox: deque[str] = deque()

    def open(self) -> None:
        if self._open:
            return
        self._open = True
        self._tele_accum = 0.0
        self._outbox.clear()
        self._clock.start()
        self._timer.start()
        self.opened.emit()
        # Стартовые строки прошивки (диагностика энкодера enc:ok) — через ту же
        # очередь, отложенно, как будто их прислала плата сразу после сброса.
        self._outbox.extend(self.sim.boot_lines())
        QTimer.singleShot(0, self._flush)

    def close(self) -> None:
        if not self._open:
            return
        self._timer.stop()
        self._open = False
        self._outbox.clear()
        self.closed.emit()

    def write_line(self, line: str) -> None:
        if not self._open:
            return
        # Ответы кладём в очередь и сливаем отложенно — чтобы не было
        # реентранси в обработчик сигнала во время самой отправки команды.
        self._outbox.extend(self.sim.handle_command(line))
        QTimer.singleShot(0, self._flush)

    def _flush(self) -> None:
        """Выдаёт всю накопленную очередь строк в порядке FIFO."""
        while self._outbox:
            self.line_received.emit(self._outbox.popleft())

    def _on_tick(self) -> None:
        dt = self._clock.restart()          # мс с прошлого тика
        if dt <= 0:
            dt = _TICK_MS
        self.sim.tick(float(dt))

        op = self.sim.op_ms
        if op <= 0:
            return
        eff = max(op, _TELE_MIN_DEBUG_MS) if self.sim.debug else op
        self._tele_accum += dt
        if self._tele_accum >= eff:
            self._tele_accum = 0.0
            # Телеметрию — в тот же outbox и сразу сливаем: тик не реентрантен,
            # а FIFO гарантирует, что ранее поставленные ответы уйдут первыми.
            self._outbox.append(self.sim.telemetry_line())
            self._flush()

    @property
    def is_open(self) -> bool:
        return self._open

    def describe(self) -> str:
        return "Симулятор"
