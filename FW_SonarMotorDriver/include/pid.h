/**
 * @file pid.h
 * @brief PID-регулятор для замкнутого управления шаговым двигателем по энкодеру.
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    uint8_t initialized;
} PID_State;

void PID_Reset(PID_State *pid);
float PID_Update(PID_State *pid, float error, float dt);
void PID_SetLimits(PID_State *pid, float min_val, float max_val);

#endif /* PID_H */
