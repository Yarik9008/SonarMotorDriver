# biss_encoder_arduino — драйвер BiSS-C для Arduino

Самодостаточная библиотека для чтения абсолютных энкодеров **LENZ IRS** (BiSS-C) через Arduino SPI API (STM32duino).

> **Статус в этом репозитории:** прошивками из `Software/` эта библиотека
> **не используется** — целевая плата работает на STM32Cube HAL, а основная
> прошивка читает энкодер собственным модулем
> [`FW_SonarMotorDriver/src/biss_c.c`](../../FW_SonarMotorDriver/src/biss_c.c)
> (SPI + DMA напрямую, без слоя портов). Библиотека лежит здесь как
> самостоятельный переносимый драйвер для плат на STM32duino.
>
> Ядро — `biss_encoder.c`, `biss_protocol.c`, `biss_models.c`, заголовки
> `biss_types.h`, `biss_protocol.h`, `biss_models.h`, `biss_encoder.h` и контракт
> порта `biss_port.h` — **побайтно совпадает** с
> [biss_encoder_stm32cube](../biss_encoder_stm32cube/). Различаются только порт
> (`biss_port_arduino.h` + `biss_port_arduino.cpp` против
> `biss_port_stm32_hal.h` + `biss_port_stm32_hal.c`), `library.json`, README
> и каталог `examples/`.
>
> Развилка «свести / оставить / удалить» одна на обе библиотеки и описана в
> [biss_encoder_stm32cube/README.md](../biss_encoder_stm32cube/README.md#что-с-этим-делать).

| Возможность | Поддержка |
|-------------|-----------|
| Блокирующее чтение | Да |
| Async (DMA) | Нет |

## Структура

```
biss_encoder_arduino/
├── include/biss_encoder/
│   ├── biss_types.h
│   ├── biss_protocol.h
│   ├── biss_models.h
│   ├── biss_port.h
│   ├── biss_encoder.h
│   └── biss_port_arduino.h
├── src/
└── examples/arduino_basic/
```

## Подключение (PlatformIO)

```ini
lib_deps =
    file://path/to/Software/lib/biss_encoder_arduino
```

## Быстрый старт

```cpp
#include <SPI.h>
#include "biss_encoder/biss_encoder.h"
#include "biss_encoder/biss_models.h"
#include "biss_encoder/biss_port_arduino.h"

biss_arduino_ctx_t ard = {
    .spi = &SPI,
    .de_pin = PB0,
    .re_pin = PB1,
    .spi_clock_hz = 750000,
};

biss_port_t port;
biss_port_arduino_fill(&port, &ard);

biss_encoder_t enc;
biss_encoder_cfg_t cfg = { .port = port, .frame = &BISS_LENZ_IRS_17BIT };
biss_encoder_init(&enc, &cfg);
```

Полный пример: [`examples/arduino_basic`](examples/arduino_basic).

```bash
cd examples/arduino_basic
pio run
```

## Ограничения

- Только блокирующее чтение (`biss_encoder_read`)
- `biss_encoder_start_read` возвращает ошибку
- Для опроса 1 кГц используйте [biss_encoder_stm32cube](../biss_encoder_stm32cube/) с DMA
