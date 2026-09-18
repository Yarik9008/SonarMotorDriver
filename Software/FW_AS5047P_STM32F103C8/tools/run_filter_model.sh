#!/bin/sh
# Прогон модели следящего фильтра угла на хосте — одной командой:
#
#     ./tools/run_filter_model.sh
#
# Собирает НАСТОЯЩИЙ src/angle_filter.c обычным gcc (без HAL и без math.h)
# вместе с tools/filter_model.c и печатает метрики, которые стоят в таблицах
# README (раздел «Проверка на модели»). Прогон детерминирован: свой ГПСЧ,
# одни и те же числа на любой машине.
#
# Параметры — те же макросы, что в platformio.ini; переопределяются
# переменными окружения, например:
#
#     FILTER_BW_HZ=20.0f ./tools/run_filter_model.sh
#
# Двоичный файл кладётся в .pio/host (каталог .pio уже в .gitignore).

set -eu

cd "$(dirname "$0")/.."

CC=${CC:-gcc}
SAMPLE_RATE_HZ=${SAMPLE_RATE_HZ:-2000U}
FILTER_BW_HZ=${FILTER_BW_HZ:-50.0f}
FILTER_DAMPING=${FILTER_DAMPING:-1.0f}
OUT=.pio/host/filter_model

mkdir -p .pio/host

"$CC" -std=c11 -O2 -Wall -Wextra -I include \
    -D SAMPLE_RATE_HZ="$SAMPLE_RATE_HZ" \
    -D FILTER_BW_HZ="$FILTER_BW_HZ" \
    -D FILTER_DAMPING="$FILTER_DAMPING" \
    src/angle_filter.c tools/filter_model.c -lm -o "$OUT"

# Windows: gcc добавляет .exe сам
[ -x "$OUT" ] || OUT="$OUT.exe"

exec "$OUT"
