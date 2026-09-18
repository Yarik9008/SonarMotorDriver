/**
 * @file  angle_filter.c
 * @brief Следящий наблюдатель угла (ФАПЧ 2-го порядка) — см. angle_filter.h.
 *
 * Математика намеренно в float: на 72 МГц один такт фильтра — это несколько
 * операций программной плавающей точки, единицы микросекунд. При опросе 2 кГц
 * это меньше процента загрузки ядра, а фиксированная точка потребовала бы
 * ручного подбора масштабов под каждую полосу.
 * Библиотека math.h НЕ используется (в сборке нет -lm).
 */
#include "angle_filter.h"

#define TWO_PI   6.28318530718f

/** Округление к ближайшему целому без math.h. */
static int32_t round_i32(float x)
{
    return (int32_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

/** Приведение разности углов к +-полоборота (аргумент по модулю < оборота). */
static float wrap_f(float d)
{
    if (d >= (float)ANGLE_FILTER_HALF) {
        d -= (float)ANGLE_FILTER_COUNTS;
    } else if (d < -(float)ANGLE_FILTER_HALF) {
        d += (float)ANGLE_FILTER_COUNTS;
    }
    return d;
}

/** Приведение оценки угла к [0, 16384) с подсчётом оборотов. */
static void normalize(angle_filter_t *f)
{
    /* За такт угол меняется меньше чем на пол-оборота (иначе измерение всё
       равно неоднозначно), поэтому цикл делает максимум одну итерацию. */
    while (f->theta >= (float)ANGLE_FILTER_COUNTS) {
        f->theta -= (float)ANGLE_FILTER_COUNTS;
        f->turns++;
    }
    while (f->theta < 0.0f) {
        f->theta += (float)ANGLE_FILTER_COUNTS;
        f->turns--;
    }
}

int32_t angle_filter_wrap(int32_t diff)
{
    diff %= ANGLE_FILTER_COUNTS;            /* C99: знак остатка = знак делимого */
    if (diff >= ANGLE_FILTER_HALF) {
        diff -= ANGLE_FILTER_COUNTS;
    } else if (diff < -ANGLE_FILTER_HALF) {
        diff += ANGLE_FILTER_COUNTS;
    }
    return diff;
}

float angle_filter_max_bandwidth(float sample_rate_hz)
{
    return (ANGLE_FILTER_WN_TS_MAX / TWO_PI) * sample_rate_hz;
}

void angle_filter_init(angle_filter_t *f, float sample_rate_hz,
                       float bandwidth_hz, float damping)
{
    float wn;
    float bw_max = angle_filter_max_bandwidth(sample_rate_hz);

    if (damping <= 0.0f) {
        damping = 1.0f;
    }
    if (bandwidth_hz <= 0.0f || bandwidth_hz > bw_max) {
        bandwidth_hz = bw_max;      /* не разгоняем контур до неустойчивости */
    }

    wn = TWO_PI * bandwidth_hz;

    f->ts           = 1.0f / sample_rate_hz;
    f->gain_p       = 2.0f * damping * wn * f->ts;   /* Kp * Ts */
    f->gain_i       = wn * wn * f->ts;               /* Ki * Ts */
    f->bandwidth_hz = bandwidth_hz;
    f->damping      = damping;
    f->gate         = (float)ANGLE_FILTER_GATE_DEFAULT;
    f->reject_limit = ANGLE_FILTER_REJECT_LIMIT_DEF;

    angle_filter_reset(f);
}

void angle_filter_set_gate(angle_filter_t *f, float gate_counts, uint32_t reject_limit)
{
    if (gate_counts > 0.0f) {
        f->gate = gate_counts;
    }
    if (reject_limit > 0U) {
        f->reject_limit = reject_limit;
    }
}

void angle_filter_reset(angle_filter_t *f)
{
    f->theta      = 0.0f;
    f->omega      = 0.0f;
    f->turns      = 0;
    f->locked     = false;
    f->raw_prev   = 0.0f;
    f->have_prev  = false;
    f->reject_run = 0U;
    f->last_err   = 0;
    f->n_update   = 0U;
    f->n_predict  = 0U;
    f->n_gated    = 0U;
    f->n_relock   = 0U;
}

/**
 * Захват: первый отсчёт только запоминаем, на втором задаём и угол, и
 * начальную скорость (по разности двух соседних отсчётов). Иначе на уже
 * вращающемся валу контур пришлось бы «разгонять» с нуля сотни тактов.
 */
static void acquire(angle_filter_t *f, uint16_t raw)
{
    if (f->have_prev) {
        f->omega  = wrap_f((float)raw - f->raw_prev) / f->ts;
        f->theta  = (float)raw;
        f->turns  = 0;
        f->locked = true;
        f->n_update++;
    }
    f->raw_prev  = (float)raw;
    f->have_prev = true;
    f->last_err  = 0;
}

void angle_filter_update(angle_filter_t *f, uint16_t raw)
{
    float e;

    raw &= (uint16_t)(ANGLE_FILTER_COUNTS - 1);

    if (!f->locked) {
        acquire(f, raw);
        return;
    }
    f->raw_prev = (float)raw;

    /* 1. Прогноз на текущий такт по последней оценке скорости. */
    f->theta += f->ts * f->omega;
    normalize(f);

    /* 2. Невязка: насколько измерение разошлось с прогнозом.
          На установившейся скорости это чистый шум датчика. */
    e = wrap_f((float)raw - f->theta);
    f->last_err = round_i32(e);

    /* 3. Отбраковка выброса. Испорченный отсчёт лучше пропустить целиком
          (остаться на прогнозе), чем «подмешать» его в интегратор скорости:
          обрезанная, но принятая поправка разогнала бы оценку скорости. */
    if (e > f->gate || e < -f->gate) {
        f->n_gated++;
        f->reject_run++;
        if (f->reject_run >= f->reject_limit) {
            /* Серия отбраковок — это не помеха, а потеря захвата
               (рывок вала, снятие/установка магнита). Захватываемся заново. */
            f->locked     = false;
            f->have_prev  = false;
            f->reject_run = 0U;
            f->n_relock++;
        }
        return;
    }
    f->reject_run = 0U;

    /* 4. Коррекция. Сначала интегратор скорости, затем угол — именно это
          даёт контур типа II с нулевой ошибкой на постоянной скорости. */
    f->omega += f->gain_i * e;
    f->theta += f->gain_p * e;
    normalize(f);

    f->n_update++;
}

void angle_filter_predict(angle_filter_t *f)
{
    if (!f->locked) {
        return;             /* ещё не захватились — вести нечего */
    }
    f->theta += f->ts * f->omega;
    normalize(f);
    f->n_predict++;
}

uint16_t angle_filter_angle(const angle_filter_t *f)
{
    int32_t a = round_i32(f->theta);

    /* округление вверх на границе оборота: 16383.7 -> 16384 -> 0 */
    if (a >= ANGLE_FILTER_COUNTS) {
        a -= ANGLE_FILTER_COUNTS;
    } else if (a < 0) {
        a += ANGLE_FILTER_COUNTS;
    }
    return (uint16_t)a;
}

uint32_t angle_filter_centideg(const angle_filter_t *f)
{
    /* 36000 / 16384 = 2.197265625 сотых градуса на отсчёт */
    int32_t cdeg = round_i32(f->theta * 2.197265625f);

    if (cdeg >= 36000) {
        cdeg -= 36000;
    } else if (cdeg < 0) {
        cdeg += 36000;
    }
    return (uint32_t)cdeg;
}

int32_t angle_filter_centideg_per_s(const angle_filter_t *f)
{
    return round_i32(f->omega * 2.197265625f);
}

int32_t angle_filter_centirpm(const angle_filter_t *f)
{
    /* об/мин = отсчёты/с * 60 / 16384; в сотых долях — ещё * 100 */
    return round_i32(f->omega * 0.3662109375f);
}

float angle_filter_noise_bandwidth(const angle_filter_t *f)
{
    float zeta = f->damping;

    /* Классический результат Гарднера: B_L [Гц] = (wn / 2) * (zeta + 1/(4*zeta)),
       где wn — в рад/с. Единицы здесь легко перепутать, поэтому: при zeta = 1
       B_L = pi * f_bw * 1.25, то есть для f_bw = 50 Гц это 196 Гц, а не 31 Гц.
       Формула сверена с моделью (tools/run_filter_model.sh, сценарий 8):
       при f_bw = 10 Гц предсказание 5.05 против измеренных 5.06, при 50 Гц —
       2.26 против 2.17. Расхождение растёт с wn*Ts (на 63 Гц уже 2.01 против
       1.90) — это искажение полосы при явной дискретизации. */
    return 0.5f * TWO_PI * f->bandwidth_hz * (zeta + 1.0f / (4.0f * zeta));
}
