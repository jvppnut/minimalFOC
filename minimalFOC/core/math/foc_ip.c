#include "foc_ip.h"

void FOC_IP_Init(FOC_IP_t *ip, float Kp, float Ki, float out_min, float out_max)
{
    ip->Kp      = Kp;
    ip->Ki      = Ki;
    ip->out_min = out_min;
    ip->out_max = out_max;
    ip->integrator = 0.0f;
}

void FOC_IP_SetGains(FOC_IP_t *ip, float Kp, float Ki)
{
    ip->Kp = Kp;
    ip->Ki = Ki;
}

void FOC_IP_Reset(FOC_IP_t *ip)
{
    ip->integrator = 0.0f;
}

float FOC_IP_Update(FOC_IP_t *ip, float ref, float meas)
{
    float error = ref - meas;

    ip->integrator = FOC_Clamp(ip->integrator + ip->Ki * error,
                               ip->out_min, ip->out_max);

    return FOC_Clamp(ip->integrator - ip->Kp * meas,
                     ip->out_min, ip->out_max);
}
