"""Транспортный слой: единый интерфейс канала связи с прошивкой."""

from .base import Transport
from .serial_transport import SerialTransport, find_stm32_port, list_serial_ports
from .sim_transport import SimTransport

__all__ = [
    "Transport",
    "SerialTransport",
    "SimTransport",
    "find_stm32_port",
    "list_serial_ports",
]
