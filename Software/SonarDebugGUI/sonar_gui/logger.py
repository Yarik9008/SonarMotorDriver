"""Logger — журнал TX/RX/INFO с привязкой по времени (порт из test_commands.py).

Пишет в файл logs/gui_YYYYMMDD_HHMMSS.log и дублирует сообщения Qt-сигналом
message(kind, text) — его слушает панель консоли.
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QObject, Signal

LOG_DIR = Path(__file__).resolve().parent.parent / "logs"


class Logger(QObject):
    message = Signal(str, str, str)     # ts 'HH:MM:SS.mmm', kind 'TX'|'RX'|'INFO'|'ERR', text

    def __init__(self, log_dir: Path | None = None):
        super().__init__()
        now = datetime.now()
        # Логи раскладываются по подпапкам-дням: logs/YYYY-MM-DD/gui_HHMMSS.log
        base = log_dir or LOG_DIR
        d = base / now.strftime("%Y-%m-%d")
        d.mkdir(parents=True, exist_ok=True)
        self.log_path = d / f"gui_{now.strftime('%H%M%S')}.log"
        self._f = open(self.log_path, "w", encoding="utf-8")
        self._f.write(f"# SonarDebugGUI log — {datetime.now().isoformat()}\n\n")
        self._f.flush()

    @staticmethod
    def _ts() -> str:
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _write(self, kind: str, text: str) -> None:
        ts = self._ts()
        self._f.write(f"[{ts}] {kind}: {text}\n")
        self._f.flush()
        self.message.emit(ts, kind, text)

    def log_tx(self, text: str) -> None:
        self._write("TX", text)

    def log_rx(self, text: str) -> None:
        self._write("RX", text)

    def log_info(self, text: str) -> None:
        self._write("INFO", text)

    def log_err(self, text: str) -> None:
        self._write("ERR", text)

    def close(self) -> None:
        try:
            self._f.close()
        except Exception:                # noqa: BLE001
            pass
