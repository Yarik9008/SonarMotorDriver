/* pid.c — Реализация PID-регулятора.
 *
 * Формула: out = kp*e + ki*integral(e) + kd*de/dt.
 * Антивинд Ap: при выходе на насыщение integral не накапливается.
 */

#include "pid.h"

/* Сброс накопленного интеграла и истории. Вызывать при смене режима (CL↔OL). */
void PID_Reset(PID_State *pid)
{
    if (!pid)
        return;
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->initialized = 0;
}

/* Обновление: вычисляет выход по ошибке и dt. Выход ограничен output_min/max. */
float PID_Update(PID_State *pid, float error, float dt)
{
    if (!pid || dt <= 0.0f)
        return 0.0f;

    pid->integral += error * dt;
    float derivative = pid->initialized ? (error - pid->prev_error) / dt : 0.0f;
    pid->prev_error  = error;
    pid->initialized = 1;

    float out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (out < pid->output_min)
        out = pid->output_min;
    else if (out > pid->output_max)
        out = pid->output_max;

    /* Антинакрутка: ограничиваем интеграл */
    if (pid->ki != 0.0f && (out == pid->output_min || out == pid->output_max))
        pid->integral -= error * dt;

    return out;
}

