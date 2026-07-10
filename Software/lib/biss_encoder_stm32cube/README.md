# biss_encoder_stm32cube — драйвер BiSS-C для STM32Cube HAL

Самодостаточная библиотека для чтения абсолютных энкодеров **LENZ IRS** (BiSS-C) по SPI через RS-485 (THVD1452).

| Возможность | Поддержка |
|-------------|-----------|
| Блокирующее чтение | Да |
| Async (DMA) | Да |

## Структура

```
biss_encoder_stm32cube/
├── include/biss_encoder/
│   ├── biss_types.h
│   ├── biss_protocol.h
│   ├── biss_models.h
│   ├── biss_port.h
│   ├── biss_encoder.h
│   └── biss_port_stm32_hal.h
├── src/
└── examples/stm32cube_basic/
```

## Подключение (PlatformIO)

```ini
lib_deps =
    file://path/to/Software/lib/biss_encoder_stm32cube
```

## Быстрый старт

```c
#include "biss_encoder/biss_encoder.h"
#include "biss_encoder/biss_models.h"
#include "biss_encoder/biss_port_stm32_hal.h"

static biss_hal_ctx_t   g_hal;
static biss_encoder_t   g_enc;

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    biss_port_stm32_hal_spi_msp_init(hspi, &g_hal);
}

g_hal.de_port = GPIOB; g_hal.de_pin = GPIO_PIN_0;
g_hal.re_port = GPIOB; g_hal.re_pin = GPIO_PIN_1;
g_hal.use_dma = 1;

biss_port_t port;
biss_port_stm32_hal_fill(&port, &g_hal);

biss_encoder_cfg_t cfg = { .port = port, .frame = &BISS_LENZ_IRS_17BIT };
biss_encoder_init(&g_enc, &cfg);
```

Полный пример: [`examples/stm32cube_basic`](examples/stm32cube_basic).

```bash
cd examples/stm32cube_basic
pio run
```
