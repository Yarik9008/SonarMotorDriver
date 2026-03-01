# SonarMotorDriver

Проект драйвера шагового двигателя с замкнутым контуром управления (PID) по абсолютному энкодеру LENZ IRS (BiSS C). Включает прошивку для STM32F103C8 и проект печатной платы.

## Структура

| Каталог | Описание |
|---------|----------|
| **FW_SonarMotorDriver/** | Прошивка STM32F103C8: PID, энкодер BiSS C, TMC2208, USB CDC + UART DMA |
| **PCB_SonarMotorDriver/**| Проект PCB (Altium) — схема и разводка |

## Возможности

- **PID** — замкнутый контур по энкодеру, автопереход в open-loop при потере связи
- **LENZ IRS** — абсолютная позиция 17–18 бит по BiSS C (SPI + DMA + THVD1452)
- **TMC2208** — шаговый драйвер (STEP/DIR/ENABLE)
- **USB CDC + UART** — команды и телеметрия (дублирование, UART через DMA)
- **IWDG** — сторожевой таймер

## Быстрый старт

```bash
cd FW_SonarMotorDriver
pio run                    # Сборка
pio run --target upload    # Прошивка (ST-Link)
```

Подробности — [FW_SonarMotorDriver/README.md](FW_SonarMotorDriver/README.md).

## Требования

- **PlatformIO** — сборка прошивки
- **ST-Link** — прошивка
- **Altium Designer** — редактирование PCB


