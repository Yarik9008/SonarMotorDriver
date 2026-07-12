# Релизные образы прошивок (Intel HEX)

Готовые образы для прошивки **STM32F103C8** (адрес загрузки `0x08000000`).

| Файл | Описание | Flash |
| ---- | -------- | ----- |
| `FW_SonarMotorDriver.hex` | Основная прошивка: PID, BiSS C, TMC2209, UART, синхронизация SYNC_OUT/SYNC_IN (`sync=`) | 47 612 B (72.7%) |
| `FW_SonarMotorDriver_Sim.hex` | Имитатор UART-протокола (без мотора и энкодера) + та же синхронизация SYNC_IN (`sync=`/`sync`) | 33 704 B (51.4%) |

## Сборка

```bash
cd Software/FW_SonarMotorDriver && pio run
cd Software/FW_SonarMotorDriver_Sim && pio run
```

После сборки скопируйте `firmware.hex` из `.pio/build/stm32f103c8/` в `Release/`.

## Прошивка (WCH-Link / OpenOCD)

```bash
cd Software/FW_SonarMotorDriver   # или FW_SonarMotorDriver_Sim
pio run --target upload
```

Или вручную:

```bash
openocd -f Software/FW_SonarMotorDriver/openocd/wch-link.cfg -f target/stm32f1x.cfg \
  -c "program Release/FW_SonarMotorDriver.hex verify reset" \
  -c "shutdown"
```

## Текущие сборки

| Параметр | `FW_SonarMotorDriver.hex` | `FW_SonarMotorDriver_Sim.hex` |
| -------- | ------------------------- | ----------------------------- |
| Дата | 2026-07-12 | 2026-07-12 |
| Git commit | `161f4a6` + функционал синхронизации SYNC_IN | тот же (одинаковый код sync) |
| Платформа | PlatformIO `ststm32` 19.4.0, `framework-stm32cubef1` 1.8.6 | та же |
| MCU | STM32F103C8T6, 72 MHz | тот же |

### SHA256 (.hex)

| Файл | SHA256 |
| ---- | ------ |
| `FW_SonarMotorDriver.hex` | `2277e3f548cbb5ac0ed38d1d0004045dc430b902e06628e3c20bface7db5b21c` |
| `FW_SonarMotorDriver_Sim.hex` | `3032381b2aa4acf8f217148c7db7d277f79ab8f198ea9b9bee0e485d08a212c7` |
