/**
 * @file  filter_model.c
 * @brief Хостовая проверка следящего фильтра угла: модель датчика + сценарии.
 *
 * Собирается и запускается на ПК обычным gcc вместе с НАСТОЯЩИМ src/angle_filter.c
 * (тот не зависит ни от HAL, ни от math.h), поэтому проверяется именно тот код,
 * который уходит в прошивку. Параметры берутся из тех же макросов, что и в
 * platformio.ini: SAMPLE_RATE_HZ, FILTER_BW_HZ, FILTER_DAMPING.
 *
 * Запуск — одной командой: tools/run_filter_model.sh (см. README, раздел
 * «Проверка на модели»). Числа, которые печатает программа, и стоят в таблицах
 * README.
 *
 * Модель датчика: истинный угол задаётся сценарием (покой, постоянная скорость,
 * постоянное ускорение), к нему добавляется гауссов шум с паспортным СКО
 * 0.068° = 3.1 отсчёта, сумма округляется до отсчёта (квантование 14 бит).
 * Генератор псевдослучайных чисел свой (xorshift64*), поэтому прогон
 * детерминирован и повторяется от запуска к запуску.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "angle_filter.h"

/* --- параметры, общие с прошивкой (platformio.ini) --- */
#ifndef SAMPLE_RATE_HZ
#define SAMPLE_RATE_HZ   2000U
#endif
#ifndef FILTER_BW_HZ
#define FILTER_BW_HZ     50.0f
#endif
#ifndef FILTER_DAMPING
#define FILTER_DAMPING   1.0f
#endif

#define FS               ((double)(SAMPLE_RATE_HZ))
#define TS               (1.0 / FS)
#define COUNTS           ((double)ANGLE_FILTER_COUNTS)
#define DEG_PER_COUNT    (360.0 / COUNTS)
#define PI_D             3.14159265358979323846

/** Паспортный шум AS5047P: 0.068° СКО = 3.1 отсчёта из 16384. */
#define NOISE_COUNTS     3.1

/** Сглаживающие фильтры для сравнения (подобраны по равному выходному шуму). */
#define MA_N             5
#define IIR_ALPHA        0.30

/* ------------------------------------------------------------------ */
/*  Генератор шума: xorshift64* + преобразование Бокса — Мюллера       */
/* ------------------------------------------------------------------ */

static uint64_t rng_state = 1;
static int      rng_have_spare;
static double   rng_spare;

static void rng_seed(uint64_t s)
{
    rng_state      = s ? s : 1u;
    rng_have_spare = 0;
}

static uint64_t rng_u64(void)
{
    uint64_t x = rng_state;

    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    rng_state = x;
    return x * 2685821657736338717ULL;
}

/** Равномерное [0, 1). */
static double rng_uniform(void)
{
    return (double)(rng_u64() >> 11) * (1.0 / 9007199254740992.0);
}

/** Нормальное (0, 1). */
static double rng_gauss(void)
{
    double u1, u2, m, a;

    if (rng_have_spare) {
        rng_have_spare = 0;
        return rng_spare;
    }
    do {
        u1 = rng_uniform();
    } while (u1 < 1e-300);
    u2 = rng_uniform();

    m = sqrt(-2.0 * log(u1));
    a = 2.0 * PI_D * u2;

    rng_spare      = m * sin(a);
    rng_have_spare = 1;
    return m * cos(a);
}

/* ------------------------------------------------------------------ */
/*  Мелкие помощники                                                   */
/* ------------------------------------------------------------------ */

/** Разность углов к ±полоборота, в отсчётах (double). */
static double wrap_counts(double d)
{
    d = fmod(d, COUNTS);
    if (d >= (double)ANGLE_FILTER_HALF) {
        d -= COUNTS;
    } else if (d < -(double)ANGLE_FILTER_HALF) {
        d += COUNTS;
    }
    return d;
}

/** Модель датчика: угол в отсчётах -> шум -> квантование 14 бит. */
static uint16_t sensor(double true_counts, double noise_rms)
{
    double v = true_counts + (noise_rms > 0.0 ? noise_rms * rng_gauss() : 0.0);
    double m = fmod(v, COUNTS);
    long   q;

    if (m < 0.0) {
        m += COUNTS;
    }
    q = lround(m);
    if (q >= ANGLE_FILTER_COUNTS) {
        q -= ANGLE_FILTER_COUNTS;
    }
    return (uint16_t)q;
}

/** Оценка контура как непрерывная величина (обороты + угол), отсчёты. */
static double loop_estimate(const angle_filter_t *f)
{
    return (double)f->turns * COUNTS + (double)f->theta;
}

typedef struct {
    double n;
    double sum;
    double sumsq;
    double maxabs;
} stat_t;

static void stat_reset(stat_t *s)
{
    memset(s, 0, sizeof(*s));
}

static void stat_add(stat_t *s, double x)
{
    s->n     += 1.0;
    s->sum   += x;
    s->sumsq += x * x;
    if (fabs(x) > s->maxabs) {
        s->maxabs = fabs(x);
    }
}

static double stat_mean(const stat_t *s)
{
    return s->n > 0.0 ? s->sum / s->n : 0.0;
}

/** СКО относительно среднего (то есть шум, без постоянного запаздывания). */
static double stat_std(const stat_t *s)
{
    double m = stat_mean(s);
    double v = s->n > 0.0 ? s->sumsq / s->n - m * m : 0.0;

    return v > 0.0 ? sqrt(v) : 0.0;
}

/* ------------------------------------------------------------------ */
/*  Сценарий 1. Постоянная скорость: контур против сглаживающих        */
/* ------------------------------------------------------------------ */

#define WARM_N   4000       /* 2 с прогрева */
#define MEAS_N   200000     /* 100 с измерения */

typedef struct {
    double raw_std;
    double ma_std,   ma_lag;
    double iir_std,  iir_lag;
    double loop_std, loop_lag;
    double loop_max;
} speed_result_t;

static void run_speed(double rpm, double bw_hz, speed_result_t *out)
{
    angle_filter_t f;
    stat_t   s_raw, s_ma, s_iir, s_loop;
    double   ma_buf[MA_N];
    double   ma_sum   = 0.0;
    double   iir      = 0.0;
    double   raw_unw  = 0.0;
    double   v        = rpm / 60.0 * COUNTS;     /* отсчётов/с */
    double   theta0   = 1000.0;
    uint16_t raw_prev = 0;
    bool     have     = false;
    long     k;
    int      i;

    rng_seed(0x5047A5047ULL);
    angle_filter_init(&f, (float)FS, (float)bw_hz, FILTER_DAMPING);
    stat_reset(&s_raw);
    stat_reset(&s_ma);
    stat_reset(&s_iir);
    stat_reset(&s_loop);
    memset(ma_buf, 0, sizeof(ma_buf));

    for (k = 0; k < WARM_N + MEAS_N; k++) {
        double   t      = (double)k * TS;
        double   true_c = theta0 + v * t;
        uint16_t raw    = sensor(true_c, NOISE_COUNTS);
        double   ma_out;

        /* развёртка сырого отсчёта в непрерывную величину */
        if (!have) {
            raw_unw = true_c + wrap_counts((double)raw - true_c);
            iir     = raw_unw;
            for (i = 0; i < MA_N; i++) {
                ma_buf[i] = raw_unw;
            }
            ma_sum = raw_unw * MA_N;
            have   = true;
        } else {
            raw_unw += wrap_counts((double)raw - (double)raw_prev);
        }
        raw_prev = raw;

        /* скользящее среднее по N точкам */
        ma_sum -= ma_buf[k % MA_N];
        ma_buf[k % MA_N] = raw_unw;
        ma_sum += raw_unw;
        ma_out  = ma_sum / MA_N;

        /* однополюсный БИХ */
        iir += IIR_ALPHA * (raw_unw - iir);

        /* следящий контур — настоящий код прошивки */
        angle_filter_update(&f, raw);

        if (k >= WARM_N) {
            stat_add(&s_raw,  raw_unw - true_c);
            stat_add(&s_ma,   ma_out  - true_c);
            stat_add(&s_iir,  iir     - true_c);
            stat_add(&s_loop, wrap_counts(loop_estimate(&f) - true_c));
        }
    }

    out->raw_std  = stat_std(&s_raw)  * DEG_PER_COUNT;
    out->ma_std   = stat_std(&s_ma)   * DEG_PER_COUNT;
    out->ma_lag   = stat_mean(&s_ma)  * DEG_PER_COUNT;
    out->iir_std  = stat_std(&s_iir)  * DEG_PER_COUNT;
    out->iir_lag  = stat_mean(&s_iir) * DEG_PER_COUNT;
    out->loop_std = stat_std(&s_loop) * DEG_PER_COUNT;
    out->loop_lag = stat_mean(&s_loop) * DEG_PER_COUNT;
    out->loop_max = s_loop.maxabs     * DEG_PER_COUNT;
}

static void scenario_compare(void)
{
    static const double rpms[] = { 0.0, 60.0, 600.0, 3000.0 };
    size_t i;

    printf("\n== 1. Постоянная скорость: контур против сглаживающих фильтров ==\n");
    printf("окно %d отсчётов (%.0f с) после прогрева %d отсчётов\n",
           MEAS_N, MEAS_N * TS, WARM_N);
    printf("| Скорость | Сырой, СКО | MA(%d) СКО / запазд. | БИХ(%.2f) СКО / запазд. | Контур %.0f Гц СКО / запазд. |\n",
           MA_N, IIR_ALPHA, (double)FILTER_BW_HZ);
    printf("|---|---|---|---|---|\n");

    for (i = 0; i < sizeof(rpms) / sizeof(rpms[0]); i++) {
        speed_result_t r;

        run_speed(rpms[i], FILTER_BW_HZ, &r);
        printf("| %.0f об/мин | %.4f | %.4f / %+.4f | %.4f / %+.4f | %.4f / %+.4f |\n",
               rpms[i], r.raw_std,
               r.ma_std, r.ma_lag, r.iir_std, r.iir_lag, r.loop_std, r.loop_lag);
    }
    printf("(СКО — разброс вокруг среднего, запаздывание — само среднее ошибки, градусы)\n");
}

/* ------------------------------------------------------------------ */
/*  Сценарий 2. Подавление шума на месте                               */
/* ------------------------------------------------------------------ */

static void scenario_noise(void)
{
    angle_filter_t f;
    speed_result_t r;
    double bn, theory;

    angle_filter_init(&f, (float)FS, FILTER_BW_HZ, FILTER_DAMPING);
    bn     = (double)angle_filter_noise_bandwidth(&f);
    theory = sqrt(FS / (2.0 * bn));

    run_speed(0.0, FILTER_BW_HZ, &r);

    printf("\n== 2. Подавление шума на месте (f_bw = %.2f Гц, zeta = %.2f) ==\n",
           (double)f.bandwidth_hz, (double)f.damping);
    printf("сырой %.4f град СКО -> контур %.4f град СКО, подавление в %.2f раза\n",
           r.raw_std, r.loop_std, r.raw_std / r.loop_std);
    printf("шумовая полоса Bn = %.2f Гц, теория sqrt(fs/2Bn) = %.2f раза, wn*Ts = %.3f\n",
           bn, theory, 2.0 * PI_D * (double)f.bandwidth_hz * TS);
}

/* ------------------------------------------------------------------ */
/*  Сценарий 3. Ошибка на постоянном ускорении (плата за тип II)       */
/* ------------------------------------------------------------------ */

/** Установившаяся ошибка контура при ускорении accel (об/с^2), градусы. */
static double accel_error_deg(double bw_hz, double accel_rev_s2, double *theory_deg)
{
    angle_filter_t f;
    stat_t   s;
    double   a    = accel_rev_s2 * COUNTS;      /* отсчётов/с^2 */
    long     warm = (long)(1.0 * FS);           /* 1 с на установление */
    long     meas = (long)(1.0 * FS);
    long     k;

    angle_filter_init(&f, (float)FS, (float)bw_hz, FILTER_DAMPING);
    stat_reset(&s);

    for (k = 0; k < warm + meas; k++) {
        double   t      = (double)k * TS;
        double   true_c = 0.5 * a * t * t;
        uint16_t raw    = sensor(true_c, 0.0);  /* без шума: нужна средняя ошибка */

        angle_filter_update(&f, raw);
        if (k >= warm) {
            stat_add(&s, wrap_counts(loop_estimate(&f) - true_c));
        }
    }

    if (theory_deg) {
        double wn = 2.0 * PI_D * (double)f.bandwidth_hz;

        *theory_deg = -(1.0 - (double)f.gain_p) * a / (wn * wn) * DEG_PER_COUNT;
    }
    return stat_mean(&s) * DEG_PER_COUNT;
}

static void scenario_accel(void)
{
    static const double accels[] = { 10.0, 100.0 };
    size_t i;

    printf("\n== 3. Ошибка на постоянном ускорении (f_bw = %.0f Гц) ==\n",
           (double)FILTER_BW_HZ);
    for (i = 0; i < sizeof(accels) / sizeof(accels[0]); i++) {
        double th;
        double m = accel_error_deg(FILTER_BW_HZ, accels[i], &th);

        printf("%6.0f об/с^2: измерено %+.4f град, теория -(1-Kp*Ts)*a/wn^2 = %+.4f град\n",
               accels[i], m, th);
    }
}

/* ------------------------------------------------------------------ */
/*  Сценарий 4. Одиночный выброс                                       */
/* ------------------------------------------------------------------ */

static void scenario_outlier(void)
{
    angle_filter_t clean, spoiled, skipped;
    double   rpm     = 600.0;
    double   v       = rpm / 60.0 * COUNTS;
    long     hit     = 2000;                 /* такт с испорченным отсчётом */
    long     k;
    double   max_dev = 0.0;
    long     settle  = -1;
    bool     same;

    angle_filter_init(&clean,   (float)FS, FILTER_BW_HZ, FILTER_DAMPING);
    angle_filter_init(&spoiled, (float)FS, FILTER_BW_HZ, FILTER_DAMPING);
    angle_filter_init(&skipped, (float)FS, FILTER_BW_HZ, FILTER_DAMPING);

    for (k = 0; k < hit + (long)(0.1 * FS); k++) {
        double   true_c = 1000.0 + v * (double)k * TS;
        uint16_t raw    = sensor(true_c, 0.0);

        angle_filter_update(&clean, raw);
        if (k == hit) {
            uint16_t bad = (uint16_t)((raw + ANGLE_FILTER_HALF) &
                                      (ANGLE_FILTER_COUNTS - 1));

            angle_filter_update(&spoiled, bad);   /* выброс на 180° */
            angle_filter_predict(&skipped);       /* такт по модели */
        } else {
            angle_filter_update(&spoiled, raw);
            angle_filter_update(&skipped, raw);
        }

        if (k >= hit) {
            double dev = fabs(wrap_counts(loop_estimate(&spoiled) -
                                          loop_estimate(&clean)));

            if (dev > max_dev) {
                max_dev = dev;
            }
            if (settle < 0 && k > hit && dev < 0.01) {
                settle = k - hit;
            }
        }
    }

    same = (spoiled.theta == skipped.theta) && (spoiled.omega == skipped.omega) &&
           (spoiled.turns == skipped.turns);

    printf("\n== 4. Одиночный выброс (скачок на 180° в одном отсчёте) ==\n");
    printf("забраковано %u, перезахватов %u, состояние совпадает с тактом по модели: %s\n",
           spoiled.n_gated, spoiled.n_relock, same ? "да (бит в бит)" : "НЕТ");
    printf("расхождение с чистым прогоном: max %.4f отсчёта (%.5f град), рассасывается за %ld тактов (%.1f мс)\n",
           max_dev, max_dev * DEG_PER_COUNT, settle,
           settle >= 0 ? (double)settle * TS * 1000.0 : 0.0);
}

/* ------------------------------------------------------------------ */
/*  Сценарий 5. Настоящий скачок вала -> перезахват                    */
/* ------------------------------------------------------------------ */

static void scenario_jump(void)
{
    angle_filter_t f;
    double   rpm         = 600.0;
    double   v           = rpm / 60.0 * COUNTS;
    long     jump_k      = 2000;
    double   jump_counts = COUNTS / 4.0;      /* 90° */
    long     k, relock_k = -1, track_k = -1;

    angle_filter_init(&f, (float)FS, FILTER_BW_HZ, FILTER_DAMPING);

    for (k = 0; k < jump_k + (long)(0.1 * FS); k++) {
        double   true_c = 1000.0 + v * (double)k * TS +
                          (k >= jump_k ? jump_counts : 0.0);
        uint16_t raw           = sensor(true_c, 0.0);
        uint32_t relock_before = f.n_relock;

        angle_filter_update(&f, raw);
        if (relock_k < 0 && f.n_relock > relock_before) {
            relock_k = k;
        }
        if (relock_k >= 0 && track_k < 0 && f.locked &&
            fabs(wrap_counts(loop_estimate(&f) - true_c)) < 1.0) {
            track_k = k;
        }
    }

    printf("\n== 5. Настоящий скачок вала на 90° ==\n");
    printf("порог отбраковки %.0f отсчётов, предел серии %u отсчётов\n",
           (double)f.gate, f.reject_limit);
    printf("забраковано до перезахвата: %ld отсчётов (%.1f мс); захват восстановлен через %ld отсчётов (%.1f мс)\n",
           relock_k - jump_k + 1, (double)(relock_k - jump_k + 1) * TS * 1000.0,
           track_k - jump_k + 1, (double)(track_k - jump_k + 1) * TS * 1000.0);
}

/* ------------------------------------------------------------------ */
/*  Сценарий 6. Потерянные отсчёты                                     */
/* ------------------------------------------------------------------ */

/** Прогон с долей p потерянных отсчётов; печатает строку статистики. */
static void dropout_run(double p, double rpm)
{
    angle_filter_t f;
    stat_t   s;
    double   v    = rpm / 60.0 * COUNTS;
    long     n    = WARM_N + MEAS_N;
    long     k;
    double   last = 0.0;

    rng_seed(0xD0D0ULL);
    angle_filter_init(&f, (float)FS, FILTER_BW_HZ, FILTER_DAMPING);
    stat_reset(&s);

    for (k = 0; k < n; k++) {
        double   true_c = 1000.0 + v * (double)k * TS;
        uint16_t raw    = sensor(true_c, NOISE_COUNTS);

        if (p > 0.0 && rng_uniform() < p) {
            angle_filter_predict(&f);
        } else {
            angle_filter_update(&f, raw);
        }
        if (k >= WARM_N) {
            last = wrap_counts(loop_estimate(&f) - true_c);
            stat_add(&s, last);
        }
    }

    printf("| %.0f%% | %u | %+.3f | %.3f | %.3f (%.4f град) | %+.3f | %u |\n",
           p * 100.0, f.n_predict, stat_mean(&s), stat_std(&s),
           s.maxabs, s.maxabs * DEG_PER_COUNT, last, f.n_relock);
}

static void scenario_dropouts(void)
{
    printf("\n== 6. Потерянные отсчёты на 600 об/мин (окно %d отсчётов) ==\n", MEAS_N);
    printf("| пропусков | тактов по модели | среднее, отсч. | СКО, отсч. | max, отсч. | в конце прогона | перезахватов |\n");
    printf("|---|---|---|---|---|---|---|\n");
    dropout_run(0.00, 600.0);
    dropout_run(0.05, 600.0);
}

/* ------------------------------------------------------------------ */
/*  Сценарий 7. Холодный захват на вращающемся валу                    */
/* ------------------------------------------------------------------ */

static void scenario_acquire(void)
{
    static const double rpms[] = { 3000.0, 12000.0, 20000.0 };
    size_t i;

    printf("\n== 7. Холодный захват на вращающемся валу ==\n");
    printf("| Скорость | отсчётов/такт | ошибка через 100 мс | max ошибка после 10 мс | перезахватов |\n");
    printf("|---|---|---|---|---|\n");

    for (i = 0; i < sizeof(rpms) / sizeof(rpms[0]); i++) {
        angle_filter_t f;
        double v      = rpms[i] / 60.0 * COUNTS;
        long   n      = (long)(0.1 * FS);
        long   k;
        double maxerr = 0.0, err = 0.0;

        rng_seed(0xC01DULL);
        angle_filter_init(&f, (float)FS, FILTER_BW_HZ, FILTER_DAMPING);

        for (k = 0; k <= n; k++) {
            double   true_c = 1000.0 + v * (double)k * TS;
            uint16_t raw    = sensor(true_c, NOISE_COUNTS);

            angle_filter_update(&f, raw);
            err = fabs(wrap_counts(loop_estimate(&f) - true_c));
            if (k >= (long)(0.01 * FS) && err > maxerr) {
                maxerr = err;
            }
        }
        printf("| %.0f об/мин | %.0f | %.2f отсчёта | %.2f отсчёта | %u |\n",
               rpms[i], v * TS, err, maxerr, f.n_relock);
    }
}

/* ------------------------------------------------------------------ */
/*  Сценарий 8. Компромисс полосы                                      */
/* ------------------------------------------------------------------ */

static void scenario_bandwidth(void)
{
    static const double bws[] = { 10.0, 20.0, 50.0, 63.0 };
    size_t i;

    printf("\n== 8. Компромисс полосы на fs = %.0f Гц ==\n", FS);
    printf("| f_bw | Шум на выходе | подавление, раз | sqrt(fs/2Bn) | Запаздывание при 100 об/с^2 | теория |\n");
    printf("|---|---|---|---|---|---|\n");
    for (i = 0; i < sizeof(bws) / sizeof(bws[0]); i++) {
        angle_filter_t f;
        speed_result_t r;
        double th;
        double lag = accel_error_deg(bws[i], 100.0, &th);

        angle_filter_init(&f, (float)FS, (float)bws[i], FILTER_DAMPING);
        run_speed(0.0, bws[i], &r);
        printf("| %.0f Гц | %.4f | %.2f | %.2f | %+.4f | %+.4f |\n",
               bws[i], r.loop_std, r.raw_std / r.loop_std,
               sqrt(FS / (2.0 * (double)angle_filter_noise_bandwidth(&f))), lag, th);
    }
}

/* ------------------------------------------------------------------ */
/*  Сценарий 9. Граничные свойства                                     */
/* ------------------------------------------------------------------ */

/**
 * Устойчив ли контур при заданном wn*Ts. Коэффициенты подставляются напрямую,
 * минуя ограничение в angle_filter_init(): иначе настоящий предел не нащупать.
 */
static bool loop_stable(double u, double zeta)
{
    angle_filter_t f;
    double err = 0.0;
    int    i;

    angle_filter_init(&f, (float)FS, 1.0f, (float)zeta);
    angle_filter_set_gate(&f, 1.0e9f, 1000000000U);
    f.gain_p = (float)(2.0 * zeta * u);         /* Kp*Ts = 2*zeta*wn*Ts */
    f.gain_i = (float)(u * u / TS);             /* Ki*Ts = (wn*Ts)^2/Ts  */

    angle_filter_update(&f, 1000);              /* захват по постоянному входу */
    angle_filter_update(&f, 1000);
    f.theta = 1000.0f + 100.0f;                 /* отклонение 100 отсчётов */
    f.omega = 0.0f;
    f.turns = 0;

    for (i = 0; i < 20000; i++) {
        angle_filter_update(&f, 1000);
        err = fabs(wrap_counts(loop_estimate(&f) - 1000.0));
        if (!isfinite(err) || err > 1.0e5 ||
            !isfinite((double)f.omega) || fabs((double)f.omega) > 1.0e9) {
            return false;                       /* расходится */
        }
    }
    return err < 1.0;                           /* отклонение рассосалось */
}

/** Перерегулирование по углу на скачке входа, % от скачка. */
static double step_overshoot(double bw_hz, double zeta)
{
    angle_filter_t f;
    double step = 400.0;                        /* меньше порога отбраковки 512 */
    double base = 1000.0;
    double peak = 0.0;
    int    k;

    angle_filter_init(&f, (float)FS, (float)bw_hz, (float)zeta);
    for (k = 0; k < 4000; k++) {                /* установка на постоянном входе */
        angle_filter_update(&f, (uint16_t)base);
    }
    for (k = 0; k < 8000; k++) {
        double d;

        angle_filter_update(&f, (uint16_t)(base + step));
        d = loop_estimate(&f) - base;
        if (d > peak) {
            peak = d;
        }
    }
    return (peak - step) / step * 100.0;
}

static void scenario_limits(void)
{
    static const double zetas[] = { 0.5, 0.707, 1.0, 2.0, 5.0 };
    static const double rpms[]  = { 0.0, 60.0, 600.0, 3000.0, 6000.0, 12000.0 };
    angle_filter_t f;
    double u, u_max = 0.0, worst = 0.0;
    size_t i;

    printf("\n== 9. Граничные свойства ==\n");

    /* 9.1 ограничение полосы в angle_filter_init() */
    angle_filter_init(&f, (float)FS, 1000.0f, FILTER_DAMPING);
    printf("9.1 angle_filter_max_bandwidth(%.0f) = %.2f Гц; заказ 1000 Гц ограничен до %.2f Гц (wn*Ts = %.3f)\n",
           FS, (double)angle_filter_max_bandwidth((float)FS), (double)f.bandwidth_hz,
           2.0 * PI_D * (double)f.bandwidth_hz * TS);

    /* 9.2 настоящий предел устойчивости по wn*Ts */
    for (u = 0.05; u <= 1.5; u += 0.005) {
        if (loop_stable(u, 1.0)) {
            u_max = u;
        } else {
            break;
        }
    }
    printf("9.2 предел устойчивости (zeta = 1, поиск с шагом 0.005): wn*Ts <= %.3f, теория 2*sqrt(2)-2 = %.3f\n",
           u_max, 2.0 * sqrt(2.0) - 2.0);
    printf("    это f_bw = %.1f Гц при fs = %.0f Гц; рабочее ограничение %.2f Гц (запас по wn*Ts в %.1f раза)\n",
           u_max / (2.0 * PI_D) * FS, FS,
           (double)angle_filter_max_bandwidth((float)FS),
           u_max / (double)ANGLE_FILTER_WN_TS_MAX);

    /* 9.3 перерегулирование по углу на скачке (проверка «zeta = 1 — без выброса») */
    printf("9.3 перерегулирование по углу на скачке входа 400 отсчётов:\n");
    printf("    | zeta | f_bw = 5 Гц (wn*Ts = 0.016) | f_bw = %.0f Гц (wn*Ts = %.3f) | Kp*Ts при %.0f Гц |\n",
           (double)FILTER_BW_HZ, 2.0 * PI_D * (double)FILTER_BW_HZ * TS,
           (double)FILTER_BW_HZ);
    printf("    |---|---|---|---|\n");
    for (i = 0; i < sizeof(zetas) / sizeof(zetas[0]); i++) {
        printf("    | %.3f | %+.2f%% | %+.2f%% | %.3f |\n", zetas[i],
               step_overshoot(5.0, zetas[i]), step_overshoot(FILTER_BW_HZ, zetas[i]),
               2.0 * zetas[i] * 2.0 * PI_D * (double)FILTER_BW_HZ * TS);
    }
    printf("    непрерывный контур при zeta = 1 даёт 1+exp(-2) = %+.2f%%: это ноль ветви Kp,\n",
           exp(-2.0) * 100.0);
    printf("    полюса при этом вещественные — колебаний нет, но выброс есть;\n");
    printf("    при Kp*Ts > 1 пропорциональная ветвь сама перескакивает измерение\n");

    /* 9.4 запаздывание на всех скоростях */
    printf("9.4 среднее смещение на постоянной скорости (градусы):");
    for (i = 0; i < sizeof(rpms) / sizeof(rpms[0]); i++) {
        speed_result_t r;

        run_speed(rpms[i], FILTER_BW_HZ, &r);
        printf("  %.0f об/мин %+.5f", rpms[i], r.loop_lag);
        if (fabs(r.loop_lag) > worst) {
            worst = fabs(r.loop_lag);
        }
    }
    printf("\n    худшее по модулю: %.5f град\n", worst);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    angle_filter_t f;

    angle_filter_init(&f, (float)FS, FILTER_BW_HZ, FILTER_DAMPING);

    printf("=== Проверка angle_filter.c на модели (хостовый прогон) ===\n");
    printf("fs = %.0f Гц, f_bw = %.2f Гц, zeta = %.2f, Kp*Ts = %.4f, Ki*Ts = %.3f 1/с\n",
           FS, (double)f.bandwidth_hz, (double)f.damping,
           (double)f.gain_p, (double)f.gain_i);
    printf("шум датчика %.2f отсчёта СКО (%.4f град) + квантование 14 бит, ГПСЧ детерминирован\n",
           NOISE_COUNTS, NOISE_COUNTS * DEG_PER_COUNT);

    scenario_compare();
    scenario_noise();
    scenario_accel();
    scenario_outlier();
    scenario_jump();
    scenario_dropouts();
    scenario_acquire();
    scenario_bandwidth();
    scenario_limits();

    printf("\n=== прогон завершён ===\n");
    return 0;
}
