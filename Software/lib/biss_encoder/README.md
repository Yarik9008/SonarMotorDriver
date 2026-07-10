# biss_encoder — драйвер BiSS-C для LENZ IRS

Библиотека для чтения абсолютных энкодеров **LENZ IRS** (и совместимых BiSS-C) по SPI через RS-485 трансивер (THVD1452).

Поддерживаются два фреймворка:

| Фреймворк | Порт | Async (DMA) |
|-----------|------|-------------|
| **STM32Cube HAL** | `biss_port_stm32_hal.c` | Да |
| **Arduino (STM32duino)** | `biss_port_arduino.cpp` | Нет (только blocking) |

## Структура

```
biss_encoder/
├── include/biss_encoder/
│   ├── biss_types.h          — статусы, biss_reading_t
│   ├── biss_protocol.h       — парсер кадра (без HAL)
│   ├── biss_models.h         — пресеты LENZ IRS 17/18 бит
│   ├── biss_port.h           — колбэки платформы
│   ├── biss_encoder.h        — публичный API
│   ├── biss_port_stm32_hal.h
│   └── biss_port_arduino.h
├── src/
├── examples/
│   ├── stm32cube_basic/
│   └── arduino_basic/
└── library.json
```

## Подключение (PlatformIO)

```ini
lib_deps =
    file://../../lib/biss_encoder
```

Или положите папку `biss_encoder` в `lib/` вашего проекта.

Порты выбираются автоматически: `USE_HAL_DRIVER` → STM32Cube, `ARDUINO` → Arduino SPI.

## API

```c
biss_status_t biss_encoder_init(biss_encoder_t *enc, const biss_encoder_cfg_t *cfg);
biss_status_t biss_encoder_read(biss_encoder_t *enc, biss_reading_t *out);   // blocking
int  biss_encoder_start_read(biss_encoder_t *enc);                            // async
int  biss_encoder_is_ready(const biss_encoder_t *enc);
biss_status_t biss_encoder_get_result(biss_encoder_t *enc, biss_reading_t *out);
void biss_encoder_abort(biss_encoder_t *enc);
```

### Пресеты моделей

| Константа | Модели | Разрешение |
|-----------|--------|------------|
| `BISS_LENZ_IRS_17BIT` | IRS-I34, I50, I60 | 17 бит (131072 counts/rev) |
| `BISS_LENZ_IRS_18BIT` | IRS-I70, I80, I90 | 18 бит (262144 counts/rev) |

## STM32Cube HAL — быстрый старт

```c
#include "biss_encoder/biss_encoder.h"
#include "biss_encoder/biss_models.h"
#include "biss_encoder/biss_port_stm32_hal.h"

static biss_hal_ctx_t   g_hal;
static biss_encoder_t   g_enc;

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    biss_port_stm32_hal_spi_msp_init(hspi, &g_hal);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    biss_port_stm32_hal_on_spi_complete(hspi, &g_hal);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    biss_port_stm32_hal_on_spi_error(hspi, &g_hal);
}

void DMA1_Channel2_IRQHandler(void) {
    biss_port_stm32_hal_dma_rx_irq(&g_hal);
}

void DMA1_Channel3_IRQHandler(void) {
    biss_port_stm32_hal_dma_tx_irq(&g_hal);
}

// init:
g_hal.de_port = GPIOB; g_hal.de_pin = GPIO_PIN_0;
g_hal.re_port = GPIOB; g_hal.re_pin = GPIO_PIN_1;
g_hal.use_dma = 1;
g_hal.dma_irq_prio = 1;

biss_port_t port;
biss_port_stm32_hal_fill(&port, &g_hal);

biss_encoder_cfg_t cfg = { .port = port, .frame = &BISS_LENZ_IRS_17BIT };
biss_encoder_init(&g_enc, &cfg);

// blocking read:
biss_reading_t rd;
biss_encoder_read(&g_enc, &rd);
```

Полный пример: [`examples/stm32cube_basic`](examples/stm32cube_basic).

Сборка:

```bash
cd examples/stm32cube_basic
pio run
```

## Arduino — быстрый старт

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

biss_reading_t rd;
biss_encoder_read(&enc, &rd);
```

Полный пример: [`examples/arduino_basic`](examples/arduino_basic).

Сборка:

```bash
cd examples/arduino_basic
pio run
```

## Аппаратная разводка (SonarMotorDriver)

| Сигнал | Пин |
|--------|-----|
| SPI1 SCK | PA5 |
| SPI1 MISO | PA6 |
| RS-485 DE | PB0 |
| RS-485 RE | PB1 |

SPI: Mode 0, MSB first, ~750 kHz (prescaler 64 при 48 МГц).

## Статусы чтения

| Код | Значение |
|-----|----------|
| `BISS_OK` | Успешное чтение |
| `BISS_ERR_CRC` | Ошибка CRC6 |
| `BISS_ERR_NO_RESPONSE` | Нет ответа энкодера |
| `BISS_ERR_SENSOR` | Бит Error в SCD |
| `BISS_ERR_WARNING` | Бит Warning в SCD |
| `BISS_ERR_SPI` | Ошибка SPI/DMA |

## Ограничения Arduino-порта

- Только **блокирующее** чтение (`biss_encoder_read`)
- `biss_encoder_start_read` возвращает ошибку (async-колбэки = NULL)
- Для опроса 1 кГц используйте STM32Cube-порт с DMA
