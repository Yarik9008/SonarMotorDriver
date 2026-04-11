# SonarMotorDriver

Проект драйвера шагового двигателя с замкнутым контуром управления (PID) по абсолютному энкодеру LENZ IRS (BiSS C). В репозитории — прошивка для **STM32F103C8**, проект печатной платы (Altium), 3D-сборка, даташиты и иллюстрации.

> **Примечание:** Подключение TMC2209 (UART, DIAG) на целевой плате **не протестировано** на аппаратуре.

## Иллюстрации

| CAD: сборка с радиатором | Плата: верхний слой (релиз 0.1) |
| --- | --- |
| ![CAD сборка с радиатором](Image/SonarMotorDriver_CAD_heatsink_isometric.png) | ![Верхний слой платы SonarMotorDriver](Image/SonarMotorDriver_PCB_top.jpg) |

## Структура репозитория

| Каталог | Описание |
|---------|----------|
| [**Software/FW_SonarMotorDriver/**](Software/FW_SonarMotorDriver/) | Основная прошивка STM32F103C8: PID, энкодер BiSS C, TMC2209, **UART** (команды и телеметрия), IWDG |
| [**Software/Test_TMC2209/**](Software/Test_TMC2209/) | Отладочная прошивка **STM32F446** + TMC2209: **USB CDC** (виртуальный COM), проверка драйвера отдельно от основной платы |
| [**Hardware/PCB_SonarMotorDriver/**](Hardware/PCB_SonarMotorDriver/) | Проект Altium Designer — схема и разводка платы |
| [**Hardware/CAD/**](Hardware/CAD/) | 3D-модели (SolidWorks/STEP): сборка с радиатором и кожухом, радиатор, STEP компонентов BOM |
| [**Docs/Datasheet/**](Docs/Datasheet/) | PDF-даташиты ключевых компонентов (STM32, TMC2209, THVD1452, питание, разъёмы и т.д.) |
| [**Image/**](Image/) | Рендеры платы и CAD для документации |

## Возможности (основная прошивка)

- **PID** — замкнутый контур по энкодеру, привязка к микрошагу, переход в open-loop при потере связи с энкодером
- **LENZ IRS** — абсолютная позиция 17–18 бит по BiSS C (SPI + DMA + THVD1452)
- **TMC2209** — STEP/DIR/ENABLE, UART для тока и микрошага
- **UART (USART1)** — текстовые команды и телеметрия (период настраивается, режимы `debug=0` / `debug=1`)
- **IWDG** — сторожевой таймер

Подробности, распиновка, протокол команд и BiSS C — в [Software/FW_SonarMotorDriver/README.md](Software/FW_SonarMotorDriver/README.md).

## Быстрый старт (сборка основной прошивки)

```bash
cd Software/FW_SonarMotorDriver
pio run                    # сборка
pio run --target upload    # прошивка (см. platformio.ini: ST-Link / WCH-Link + OpenOCD)
```

Тестовая прошивка с USB CDC: [Software/Test_TMC2209/README.md](Software/Test_TMC2209/README.md).

## Требования

- **PlatformIO** — сборка прошивок в `Software/`
- **Программатор** — ST-Link, CMSIS-DAP или иной, совместимый с настройками `upload` в `platformio.ini`
- **Altium Designer** — правка схемы и платы в `Hardware/PCB_SonarMotorDriver/`
