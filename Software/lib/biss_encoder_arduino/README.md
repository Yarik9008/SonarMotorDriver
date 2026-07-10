# biss_encoder_arduino — драйвер BiSS-C для Arduino

Самодостаточная библиотека для чтения абсолютных энкодеров **LENZ IRS** (BiSS-C) через Arduino SPI API (STM32duino).

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
