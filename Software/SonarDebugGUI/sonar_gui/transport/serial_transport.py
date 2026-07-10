"""SerialTransport — реальный COM-порт (pyserial) в отдельном потоке.

Чтение — в QThread-воркере (эмитит целые строки, фрейминг перенесён из
FW_SonarMotorDriver/tools/test_commands.py). Запись — из GUI-потока под Lock
(pyserial допускает одновременные чтение и запись из разных потоков).
"""
from __future__ import annotations

import threading

from PySide6.QtCore import QThread, Signal

from .base import Transport
from ..protocol import BAUD

try:
    import serial
    from serial.tools import list_ports
except ImportError:                     # pragma: no cover
    serial = None
    list_ports = None


def _friendly(exc: Exception) -> str:
    msg = str(exc)
    low = msg.lower()
    if "permissionerror" in low or "access is denied" in low or "занят" in low:
        return (f"{msg}\nПорт занят. Закройте Serial Monitor PlatformIO, "
                f"PuTTY и другие терминалы.")
    if "could not open" in low or "filenotfounderror" in low:
        return f"{msg}\nПорт не найден. Проверьте подключение USB-UART адаптера."
    return msg


def list_serial_ports() -> list[tuple[str, str]]:
    """[(device, description), ...] всех доступных COM-портов."""
    if list_ports is None:
        return []
    out = []
    for p in list_ports.comports():
        out.append((p.device, p.description or "?"))
    return out


def find_stm32_port() -> str | None:
    """Автоопределение COM-порта STM32 / USB-UART (порт из test_commands.py)."""
    if list_ports is None:
        return None
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "0483" in hwid or "stm" in desc or "stm32" in desc or "ch340" in hwid or "cp210" in hwid:
            return p.device
    return None


class _SerialReader(QThread):
    """Фоновое чтение порта: копит байты и эмитит целые строки."""
    line = Signal(str)
    opened = Signal()
    closed = Signal()
    error = Signal(str)

    def __init__(self, port: str, lock: threading.Lock):
        super().__init__()
        self._port = port
        self._lock = lock
        self._ser = None
        self._running = False

    def run(self) -> None:
        try:
            self._ser = serial.Serial(self._port, BAUD, timeout=0.05)
        except Exception as e:                      # noqa: BLE001
            self.error.emit(_friendly(e))
            return

        self._running = True
        self.opened.emit()
        buf = b""
        while self._running:
            try:
                n = self._ser.in_waiting
                if n:
                    buf += self._ser.read(n)
                    buf = self._emit_lines(buf)
                else:
                    self.msleep(5)
            except Exception as e:                  # noqa: BLE001
                self.error.emit(_friendly(e))
                break

        try:
            if self._ser:
                self._ser.close()
        except Exception:                           # noqa: BLE001
            pass
        self.closed.emit()

    def _emit_lines(self, buf: bytes) -> bytes:
        while True:
            pos_cr = buf.find(b"\r")
            pos_lf = buf.find(b"\n")
            if pos_cr == -1 and pos_lf == -1:
                return buf
            if pos_cr >= 0 and (pos_lf < 0 or pos_cr <= pos_lf):
                sep = b"\r"
            else:
                sep = b"\n"
            part, _, buf = buf.partition(sep)
            if part:
                text = part.decode("utf-8", errors="replace").strip()
                if text:
                    self.line.emit(text)

    def write(self, data: bytes) -> None:
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.write(data)

    def stop(self) -> None:
        self._running = False


class SerialTransport(Transport):
    def __init__(self, port: str):
        super().__init__()
        self._port = port
        self._lock = threading.Lock()
        self._reader: _SerialReader | None = None
        self._open = False

    def open(self) -> None:
        if serial is None:
            self.error.emit("Не установлен pyserial: pip install pyserial")
            return
        if self._reader is not None:
            return
        self._reader = _SerialReader(self._port, self._lock)
        self._reader.line.connect(self.line_received)
        self._reader.opened.connect(self._on_opened)
        self._reader.closed.connect(self._on_closed)
        self._reader.error.connect(self._on_error)
        self._reader.start()

    def _on_opened(self) -> None:
        self._open = True
        self.opened.emit()

    def _on_closed(self) -> None:
        self._open = False
        self.closed.emit()

    def _on_error(self, msg: str) -> None:
        self._open = False
        self.error.emit(msg)

    def close(self) -> None:
        if self._reader is not None:
            self._reader.stop()
            self._reader.wait(2000)
            self._reader = None
        if self._open:
            self._open = False
            self.closed.emit()

    def write_line(self, line: str) -> None:
        if self._reader is not None:
            raw = line if line.endswith("\r\n") else line + "\r\n"
            self._reader.write(raw.encode("utf-8"))

    @property
    def is_open(self) -> bool:
        return self._open

    def describe(self) -> str:
        return f"COM {self._port} @ {BAUD}"
