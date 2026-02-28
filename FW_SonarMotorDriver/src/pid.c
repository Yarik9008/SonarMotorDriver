/**
 * @file pid.c
 * @brief Реализация PID-регулятора.
 */

#include "pid.h"

void PID_Reset(PID_State *pid)
{
    if (!pid)
        return;
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->initialized = 0;
}

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

    /* Anti-windup: ограничиваем интеграл */
    if (pid->ki != 0.0f && (out == pid->output_min || out == pid->output_max))
        pid->integral -= error * dt;

    return out;
}

void PID_SetLimits(PID_State *pid, float min_val, float max_val)
{
    if (!pid)
        return;
    pid->output_min = min_val;
    pid->output_max = max_val;
}
