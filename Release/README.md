# Релизные образы прошивок (Intel HEX)

Готовые образы для прошивки **STM32F103C8** (адрес загрузки `0x08000000`).

| Файл | Описание | Flash |
| ---- | -------- | ----- |
| `FW_SonarMotorDriver.hex` | Основная прошивка: PID, BiSS C, TMC2209, UART | 47 208 B (72.0%) |
| `FW_SonarMotorDriver_Sim.hex` | Имитатор UART-протокола (без мотора и энкодера) | 33 332 B (50.9%) |

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

## Сборка от 2026-07-10

| Параметр | Значение |
| -------- | -------- |
| Git commit | `dfad494` |
| Платформа | PlatformIO `ststm32` 19.4.0, `framework-stm32cubef1` 1.8.6 |
| MCU | STM32F103C8T6, 72 MHz |

### SHA256 (.hex)

| Файл | SHA256 |
| ---- | ------ |
| `FW_SonarMotorDriver.hex` | `ed5e24a5241c7f8425fb3343f473744d8dbb70db95bcd87bbe366ee64b28e6be` |
| `FW_SonarMotorDriver_Sim.hex` | `c760da72f9502219022c26e39c52b6af7e45ca596fc3a59aa4a9ba1aa7ef7ecd` |
