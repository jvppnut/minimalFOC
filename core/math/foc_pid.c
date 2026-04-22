#include "foc_pid.h"

void FOC_PID_Init(FOC_PID_t *pid, float Kp, float Ki, float Kd,
                  float out_min, float out_max)
{
    pid->Kp      = Kp;
    pid->Ki      = Ki;
    pid->Kd      = Kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integrator = 0.0f;
}

void FOC_PID_SetGains(FOC_PID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void FOC_PID_Reset(FOC_PID_t *pid)
{
    pid->integrator = 0.0f;
}

float FOC_PID_Update(FOC_PID_t *pid, float error, float meas_dot)
{
    float p_out = pid->Kp * error;

    pid->integrator = FOC_Clamp(pid->integrator + pid->Ki * error,
                                pid->out_min, pid->out_max);

    /* Negative sign: D on measurement opposes the direction of change. */
    float d_out = -pid->Kd * meas_dot;

    return FOC_Clamp(p_out + pid->integrator + d_out,
                     pid->out_min, pid->out_max);
}
