"""Общая обвязка юнит-тестов SonarDebugGUI (тестов не содержит).

Делает три вещи, нужные всем тестовым модулям:

1. Ставит каталог Software/SonarDebugGUI на путь импорта — чтобы `sonar_gui`
   находился и при запуске `python -m unittest discover -s tests`, и при
   запуске одного файла из каталога tests.
2. Переводит консоль в UTF-8: в стандартной консоли Windows (cp866/cp1251)
   первое же русское сообщение об ошибке уронило бы прогон с
   UnicodeEncodeError вместо того, чтобы показать причину.
3. Даёт класс Sim — прогон модели прошивки на виртуальных часах.

Виртуальные часы важны принципиально: шаг модели 1 мс равен периоду главного
цикла прошивки (POLL_FREQ_HZ = 1 кГц), поэтому счётчик периода телеметрии
op=, пауза delay в точке скана и профиль движения считаются в тех же
единицах, что на плате, а тесты идут за микросекунды и без time.sleep.

Зависимостей, кроме стандартной библиотеки, нет: PySide6 в тестах не
импортируется (sonar_gui.protocol и sonar_gui.simulator от Qt не зависят).
"""
from __future__ import annotations

import os
import sys

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
GUI_DIR = os.path.dirname(_TESTS_DIR)                  # Software/SonarDebugGUI
REPO_ROOT = os.path.dirname(os.path.dirname(GUI_DIR))  # корень репозитория

if GUI_DIR not in sys.path:
    sys.path.insert(0, GUI_DIR)

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError, OSError):
        pass    # перенаправленный в файл/конвейер поток трогать не обязательно

from sonar_gui import protocol as P                     # noqa: E402
from sonar_gui import simulator as S                    # noqa: E402
from sonar_gui.simulator import FirmwareSimulator       # noqa: E402

TICK_MS = 1.0           # шаг модели = период главного цикла прошивки (1 кГц)
DEADBAND = S.DEADBAND_DEG


class Sim:
    """Модель прошивки с виртуальными часами и накоплением телеметрии.

    Команды идут в модель как есть (`send`), время двигается тиками по 1 мс
    (`run`). Всё, что модель за это время выдала в «UART», копится в
    `frames` — тесты считают кадры и сверяют их содержимое.
    """

    def __init__(self, *setup: str) -> None:
        self.fw = FirmwareSimulator()
        self.t_ms = 0.0
        self.frames: list[str] = []
        for cmd in setup:
            self.ok(cmd)

    # ── Команды ────────────────────────────────────────────────────────────
    def send(self, cmd: str) -> list[str]:
        """Строки ответа модели (пустой список = «молчание» прошивки).

        Команда уходит с терминатором CR+LF — ровно так её шлёт хост
        (protocol_test.Link.exchange: `write((cmd + "\\r\\n"))`), а без
        терминатора прошивка команду и не увидит: сборщик оставит её в
        приёмном кольце (line_reader.c:39-46). Кусок потока как есть, без
        дописанного конца строки, отправляет send_raw().
        """
        return self.fw.handle_command(cmd + "\r\n")

    def send_raw(self, chunk: str) -> list[str]:
        """Кусок потока байт-в-байт: терминатор (если нужен) ставит тест сам."""
        return self.fw.handle_command(chunk)

    def ok(self, cmd: str) -> str:
        """Команда, от которой ждём ровно один ответ ok: (иначе тест падает)."""
        rep = self.send(cmd)
        assert len(rep) == 1 and rep[0].startswith("ok:"), (cmd, rep)
        return rep[0]

    # ── Время ──────────────────────────────────────────────────────────────
    def run(self, ms: float, telemetry: bool = True, on_tick=None) -> None:
        """Прогоняет ms миллисекунд модельного времени.

        telemetry=False — не вызывать telemetry_tick(): так проверяется
        поведение прошивки, когда кадр взведён, но выдать его ещё не удалось
        (очередь TX занята).
        """
        for _ in range(int(round(ms / TICK_MS))):
            self.fw.tick(TICK_MS)
            if telemetry:
                line = self.fw.telemetry_tick(TICK_MS)
                if line is not None:
                    self.frames.append(line)
            self.t_ms += TICK_MS
            if on_tick is not None:
                on_tick(self.fw)

    def run_until(self, pred, limit_ms: float = 20000.0,
                  telemetry: bool = True) -> float:
        """Крутит модель, пока pred(fw) не станет истиной. Возвращает мс.

        По исчерпании лимита возвращает float('inf') — тест сам решает, что
        это значит (для sync=1, например, «не уехал» и есть ожидаемое).
        """
        spent = 0.0
        while spent < limit_ms:
            if pred(self.fw):
                return spent
            self.run(TICK_MS, telemetry=telemetry)
            spent += TICK_MS
        return float("inf") if not pred(self.fw) else spent

    def settle(self, limit_ms: float = 20000.0, telemetry: bool = True) -> float:
        """Ждёт остановки вала в цели. Возвращает затраченное время, мс."""
        return self.run_until(lambda fw: not fw.is_moving(), limit_ms, telemetry)

    # ── Наблюдение ─────────────────────────────────────────────────────────
    def err(self) -> float:
        """Ошибка до цели по кратчайшему пути, градусы."""
        return P.wrap180(self.fw.target_deg - self.fw.cur_deg)

    def events(self) -> list[str]:
        """Накопленные событийные кадры (метка ev:1)."""
        return [f for f in self.frames if f.endswith(",ev:1")]

    def periodic(self) -> list[str]:
        """Накопленные периодические кадры (без метки ev:1)."""
        return [f for f in self.frames if not f.endswith(",ev:1")]

    def scan_points(self, ms: float) -> list[float]:
        """Точки скана, на которых модель постояла за ms, по порядку.

        Точка засчитывается в состоянии SCAN_DELAY — то есть когда вал уже
        пришёл в неё (на плате в этот момент SYNC_OUT уходит в HIGH).
        """
        seen: list[float] = []

        def watch(fw):
            if fw.scan_st != S.SCAN_DELAY:
                return
            pt = round(fw.scan_sp, 6)
            if not seen or seen[-1] != pt:
                seen.append(pt)

        self.run(ms, on_tick=watch)
        return seen

    def path(self, ms: float) -> list[float]:
        """Пройденный путь: список приращений позиции по кратчайшему пути."""
        deltas: list[float] = []
        prev = [self.fw.cur_deg]

        def watch(fw):
            deltas.append(P.wrap180(fw.cur_deg - prev[0]))
            prev[0] = fw.cur_deg

        self.run(ms, on_tick=watch)
        return deltas

    def pulse_sync_in(self) -> None:
        """Фронт LOW->HIGH на входе SYNC_IN — как SyncIn_Tick() прошивки.

        Физического пина у модели нет, поэтому фронт вносится тестом: он
        взводит триггер перехода и увеличивает счётчик фронтов (поле n= в
        ответе на запрос sync).
        """
        self.fw.syncin_trig = 1
        self.fw.sync_edges += 1
