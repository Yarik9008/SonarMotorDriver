"""Transport — абстракция канала связи с прошивкой.

Одинаковый интерфейс для реального COM-порта (SerialTransport) и для
встроенного симулятора (SimTransport). UI и DeviceClient знают только про него.
"""
from __future__ import annotations

from PySide6.QtCore import QObject, Signal


class Transport(QObject):
    line_received = Signal(str)   # пришла целая строка (без CR/LF)
    opened = Signal()             # канал открыт
    closed = Signal()             # канал закрыт
    error = Signal(str)           # ошибка открытия/связи (текст для пользователя)

    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def write_line(self, line: str) -> None:
        raise NotImplementedError

    @property
    def is_open(self) -> bool:
        return False

    def describe(self) -> str:
        return self.__class__.__name__
