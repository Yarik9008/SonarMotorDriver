"""DeviceState — единая модель состояния устройства для всех индикаторов UI."""
from __future__ import annotations

from dataclasses import dataclass

from PySide6.QtCore import QObject, Signal


@dataclass
class DeviceState:
    """Снимок состояния. None = значение ещё не подтверждено устройством."""

    connected: bool = False
    channel: str = "НЕ ПОДКЛЮЧЕНО"
    channel_warn: bool = False

    cp: float | None = None
    tp: float | None = None
    pe: float | None = None
    u: float | None = None
    m: str | None = None
    ec: int | None = None
    drp: int | None = None

    # Подтверждённые параметры из телеметрии (debug=1)
    kp: float | None = None
    ki: float | None = None
    kd: float | None = None
    v: float | None = None
    a: float | None = None
    op_ms: int | None = None

    has_telemetry: bool = False


class DeviceStateModel(QObject):
    """Хранит DeviceState и рассылает сигналы при изменении (только GUI-поток)."""

    state_changed = Signal(object)       # DeviceState — полный снимок
    connected_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._st = DeviceState()

    @property
    def state(self) -> DeviceState:
        return self._st

    def set_connection(self, connected: bool, channel: str = "",
                       warn: bool = False) -> None:
        self._st.connected = connected
        if channel:
            self._st.channel = channel
        self._st.channel_warn = warn
        self.connected_changed.emit(connected)
        self._emit()

    def apply_telemetry(self, data: dict) -> None:
        if "cp" not in data:
            return
        self._st.has_telemetry = True
        float_keys = ("cp", "tp", "pe", "u", "kp", "ki", "kd", "v", "a")
        int_keys = ("ec", "drp", "op")
        for key in float_keys:
            if key in data:
                setattr(self._st, key, float(data[key]))
        for key in int_keys:
            if key in data:
                setattr(self._st, key, int(data[key]))
        if "m" in data:
            self._st.m = str(data["m"])
        self._emit()

    def set_confirmed(self, key: str, value: float | int) -> None:
        if hasattr(self._st, key):
            setattr(self._st, key, value)
            self._emit()

    def reset(self, *, keep_connection: bool = False) -> None:
        channel = self._st.channel
        warn = self._st.channel_warn
        connected = self._st.connected
        self._st = DeviceState()
        if keep_connection:
            self._st.connected = connected
            self._st.channel = channel
            self._st.channel_warn = warn
        self._emit()

    def _emit(self) -> None:
        self.state_changed.emit(self._st)
