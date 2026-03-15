/* pid.h — PID-регулятор для замкнутого управления шаговым двигателем по энкодеру. */

#ifndef PID_H
#define PID_H

#include <stdint.h>

/* Состояние PID-регулятора */
typedef struct {
    float kp;           /* пропорциональный коэффициент */
    float ki;           /* интегральный коэффициент */
    float kd;           /* дифференциальный коэффициент */
    float integral;     /* накопленный интеграл */
    float prev_error;   /* предыдущая ошибка (для D) */
    float output_min;   /* нижний предел выхода */
    float output_max;   /* верхний предел выхода */
    uint8_t initialized; /* признак инициализации (для первого шага D=0) */
} PID_State;

/* Сброс интеграла и prev_error */
void PID_Reset(PID_State *pid);

/* Обновление регулятора. error — текущая ошибка, dt — шаг по времени (с). */
float PID_Update(PID_State *pid, float error, float dt);

#endif /* PID_H */
