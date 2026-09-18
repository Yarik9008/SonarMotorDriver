# SonarMotorDriver

SonarMotorDriver — драйвер шагового двигателя с замкнутым контуром позиционирования (PID) по абсолютному энкодеру LENZ IRS (BiSS C), предназначенный для наведения гидроакустической антенны. В репозитории — прошивка для **STM32F103C8**, 3D-модель платы, даташиты компонентов и иллюстрации.

> **Статус:** управление TMC2209 по UART (ток, микрошаг, вращение) проверено на аппаратуре целевой платы — мотор вращается, энкодер отвечает, драйвер подтверждает конфигурацию (`mcfg`). Пин **DIAG** (StallGuard/Open Load) разведён, но программно не опрашивается.

## Иллюстрации

| CAD: сборка с радиатором | CAD: сборка с прозрачным кожухом |
| --- | --- |
| ![CAD сборка с радиатором](Image/SonarMotorDriver_CAD_heatsink_isometric.png) | ![CAD сборка с прозрачным кожухом](Image/SonarMotorDriver_CAD_transparent_cover_isometric.png) |

| Плата: верхний слой (релиз 0.1) | Плата: нижний слой (релиз 0.1) |
| --- | --- |
| ![Верхний слой платы SonarMotorDriver](Image/SonarMotorDriver_PCB_top.jpg) | ![Нижний слой платы SonarMotorDriver](Image/SonarMotorDriver_PCB_bottom.jpg) |

## Структура репозитория

| Каталог | Описание |
|---------|----------|
| [**Software/FW_SonarMotorDriver/**](Software/FW_SonarMotorDriver/) | Основная прошивка STM32F103C8: PID, энкодер BiSS C, TMC2209, **UART** (команды и телеметрия), синхронизация SYNC_OUT/SYNC_IN, IWDG |
| [**Software/FW_SonarMotorDriver_Sim/**](Software/FW_SonarMotorDriver_Sim/) | Имитатор основной прошивки: тот же UART-протокол (команды и телеметрия, те же два порта USART1 + USART3), но без реального энкодера и TMC2209 — для отладки ПО верхнего уровня без подключённого мотора. Отличим по boot-баннеру: `fw=1.0.0-sim` |
| [**Software/FW_Test_TMC2209/**](Software/FW_Test_TMC2209/) | Отладочная прошивка **STM32F103C8** + TMC2209: **UART CLI** через внешний USB-UART переходник, проверка драйвера отдельно от основной платы |
| [**Software/FW_AS5047P_STM32F103C8/**](Software/FW_AS5047P_STM32F103C8/) | Стенд магнитного энкодера **AS5047P** (WeAct STM32F1 Core Board): опрос по SPI на 2 кГц, следящий фильтр угла 2-го порядка, лог по **USB CDC**. Отдельное исследование альтернативного датчика — в основную прошивку его код не входит |
| [**Software/SonarDebugGUI/**](Software/SonarDebugGUI/) | Графическое приложение (PySide6) для отладки основной прошивки: элемент интерфейса на каждую команду UART-протокола кроме `diag` (он — из вкладки «Консоль»), опрос состояния платы при подключении (`mcfg`, `sync`, `om`, `hold`), PPI-диаграмма антенны, живая телеметрия, встроенный симулятор — работает без железа. Там же юнит-тесты протокола и модели прошивки ([tests/](Software/SonarDebugGUI/tests/)) |
| [**Software/lib/sonar_proto/**](Software/lib/sonar_proto/) | Общий слой командного протокола (`uart.c` + `line_reader.c` + `cmd_parser.c`) — одна копия на основную прошивку и имитатор |
| [**Software/lib/tmc2209/**](Software/lib/tmc2209/) | Общий драйвер TMC2209 (ядро протокола PDN_UART, порт STM32 HAL, фасад мотора) — одна копия на основную прошивку и тестовый стенд `FW_Test_TMC2209` |
| [**Software/lib/biss_encoder_stm32cube/**](Software/lib/biss_encoder_stm32cube/) | Драйвер BiSS-C для LENZ IRS (STM32Cube HAL, SPI + DMA). **Прошивками репозитория не используется** — в основной прошивке свой `src/biss_c.c`. Что с этими двумя библиотеками делать (свести / оставить / удалить) — решение не принято, развилка описана в [их README](Software/lib/biss_encoder_stm32cube/README.md#что-с-этим-делать) |
| [**Software/lib/biss_encoder_arduino/**](Software/lib/biss_encoder_arduino/) | Драйвер BiSS-C для LENZ IRS (Arduino / STM32duino). **Прошивками репозитория не используется**; ядро (`biss_encoder.c`, `biss_protocol.c`, `biss_models.c`, `biss_types.h`) побайтно совпадает со stm32cube-версией, различаются только порты |
| [**Docs/**](Docs/) | Протокол UART: полное описание [Protocol.md](Docs/Protocol.md), шпаргалка на страницу [Protocol_QuickStart.md](Docs/Protocol_QuickStart.md); стыковка с внешним оборудованием — [Sync.md](Docs/Sync.md) |
| [**Docs/Datasheet/**](Docs/Datasheet/) | PDF-даташиты ключевых компонентов (STM32, TMC2209, THVD1452, питание, разъёмы и т.д.) |
| [**Hardware/CAD/**](Hardware/CAD/) | 3D-модели (SolidWorks/STEP): сборка с радиатором и кожухом, радиатор, STEP-файлы компонентов из BOM |
| [**Release/**](Release/) | Готовые образы прошивок (Intel HEX) — основная и имитатор, с SHA256 и параметрами сборки |
| [**Image/**](Image/) | Рендеры платы и CAD для документации |

## Возможности (основная прошивка)

- **PID** — замкнутый контур по энкодеру, привязка к микрошагу, переход в open-loop при потере связи с энкодером
- **LENZ IRS** — абсолютная позиция 17–18 бит по BiSS C (SPI + DMA + THVD1452)
- **TMC2209** — STEP/DIR/ENABLE, UART для тока и микрошага
- **UART (USART1 + USART3)** — текстовые команды и телеметрия (период настраивается, режимы `debug=0` / `debug=1`). Два порта с одним протоколом: TX дублируется в оба, RX объединяется в один поток команд; так же устроен и имитатор
- **Сканирование сектора** — `scan=<от>,<до>,<шаг>,<пауза>`, зигзаг с паузой на каждой точке
- **Синхронизация с внешним оборудованием** — SYNC_OUT (точка достигнута) и внешний триггер SYNC_IN; источник перехода к следующей точке скана выбирается командой `sync=0/1/2` (таймер / фронт SYNC_IN / что раньше), см. [Docs/Sync.md](Docs/Sync.md)
- **Режим выдачи телеметрии `om=0/1/2`** — по таймеру `op=`, по приходу в целевую позицию (кадр помечается `,ev:1`) или оба сразу: можно получать ровно один кадр на точку скана вместо непрерывного потока
- **Снятие удержания вала `hold=0/1`** — на время замера обмотки обесточиваются (цель и точка скана сохраняются, пауза скана заморожена), `hold=1` возвращает ток
- **Запросы состояния `om` и `hold`** — ключевое слово без `=` возвращает текущее значение одной строкой (`om=1`, `hold=0`) без префикса `ok:`, как `sync` и `mcfg`. Нужны хосту после подключения: плата могла перезагрузиться или работать с другим хостом. Поддержаны одинаково основной прошивкой и имитатором
- **IWDG** — сторожевой таймер

Подробности, распиновка, протокол команд и BiSS C — в [Software/FW_SonarMotorDriver/README.md](Software/FW_SonarMotorDriver/README.md).
Протокол целиком — [Docs/Protocol.md](Docs/Protocol.md), краткая шпаргалка для интеграции — [Docs/Protocol_QuickStart.md](Docs/Protocol_QuickStart.md).

## Быстрый старт (сборка основной прошивки)

```bash
cd Software/FW_SonarMotorDriver
pio run                    # сборка
pio run --target upload    # прошивка (см. platformio.ini: ST-Link / WCH-Link + OpenOCD)
```

Имитатор, тестовый стенд TMC2209 и стенд AS5047P собираются так же, из своих каталогов — команды и подробности в README каждого из них (см. таблицу выше). GUI-отладчик [SonarDebugGUI](Software/SonarDebugGUI/) запускается отдельно, без PlatformIO. Прошивать без сборки можно готовыми образами из [Release/](Release/).

## Тесты (без железа)

```bash
cd Software/SonarDebugGUI
python -m unittest discover -s tests   # юнит-тесты протокола и модели прошивки
python protocol_test.py --sim          # конформанс-тест протокола на модели прошивки
```

Оба прогона идут на виртуальных часах, не требуют ни платы, ни COM-порта, ни
сторонних пакетов (только стандартная библиотека Python). Юнит-тестов сейчас
169, проверок конформанса — 184 (замер 2026-09-18; числа печатают сами
команды выше — `Ran 169 tests … OK` и `ИТОГО: 184/184 пройдено`).

Один из модулей юнит-тестов (`tests/test_cmd_parser_host.py`, 15 тестов)
собирает хостовым gcc **настоящий** парсер прошивки
([`Software/lib/sonar_proto`](Software/lib/sonar_proto/README.md)) и сверяет
его с моделью. Без компилятора C в PATH класс этого модуля пропускается —
тогда прогон печатает `Ran 157 tests … OK (skipped=1)`, и это не ошибка.

Те же шаги плюс сборку всех четырёх прошивок должен выполнять CI —
[.github/workflows/ci.yml](.github/workflows/ci.yml). **Workflow ещё ни разу
не запускался:** файл лежит в рабочем дереве, но не закоммичен и на origin его
нет, так что GitHub Actions его пока не видит. Все числа выше получены
локальным прогоном, а не раннером.

Автотест команд на реальной плате (нужен COM-порт и `pyserial`) —
[Software/FW_SonarMotorDriver/tools/test_commands.py](Software/FW_SonarMotorDriver/tools/test_commands.py).

## Требования

- **PlatformIO** — сборка прошивок в `Software/` (проверено на PlatformIO Core 6.2.0)
- **Программатор** — ST-Link, CMSIS-DAP или иной, совместимый с настройками `upload` в `platformio.ini`
- **Python 3.10+** (локально проверено на 3.14.7; в описании CI указан 3.12,
  но сам CI ещё ни разу не запускался — см. «Тесты» выше):
  - для [SonarDebugGUI](Software/SonarDebugGUI/) — `pip install -r Software/SonarDebugGUI/requirements.txt` (PySide6, pyqtgraph, pyserial);
  - для тестов (`unittest discover -s tests`, `protocol_test.py --sim`) — без зависимостей, хватает чистого Python;
  - для скриптов работы с платой (`tools/*.py`) — `pyserial`.
  Сборки прошивок Python-пакеты не требуют (PlatformIO ставит свои сам).
