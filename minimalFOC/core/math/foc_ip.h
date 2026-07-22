#ifndef FOC_IP_H
#define FOC_IP_H

#include "foc_math.h"

/*
 * I-P (Integral-Proportional) controller.
 *
 * Standard PI:  u = Kp*(ref-meas) + Ki*integral(ref-meas)
 *   -> reference-to-output has a zero at s = -Ki/Kp.
 *
 * I-P:          u = Ki*integral(ref-meas) - Kp*meas
 *   -> the P term acts on the measurement only, never sees the reference
 *      directly. With the same Kp, Ki this yields the IDENTICAL closed-loop
 *      characteristic equation (same poles, same L*s^2+(R+Kp)*s+Ki denominator
 *      for a current loop) but the reference-to-output numerator is just Ki,
 *      so there is no zero. Useful for isolating whether an observed
 *      overshoot/settling mismatch comes from the PI zero or from the poles
 *      themselves (plant model, loop delay, etc.).
 *
 * Anti-windup: the integrator is clamped to [out_min, out_max] each step,
 * same convention as FOC_PID_t.
 */
typedef struct {
    float Kp;       /* Proportional gain (applied to -meas)             */
    float Ki;       /* Integral gain (discrete-scaled)                  */
    float out_min;  /* Output lower limit                               */
    float out_max;  /* Output upper limit                               */

    /* State — reset with FOC_IP_Reset() on enable or mode switch. */
    float integrator;
} FOC_IP_t;

/* Initialise gains and limits, and reset internal state. */
void  FOC_IP_Init(FOC_IP_t *ip, float Kp, float Ki, float out_min, float out_max);

/* Update gains without touching the integrator.
   Use for on-the-fly tuning so the controller continues bumplessly. */
void  FOC_IP_SetGains(FOC_IP_t *ip, float Kp, float Ki);

/* Clear integrator without changing gains or limits. */
void  FOC_IP_Reset(FOC_IP_t *ip);

/*
 * Run one control step; returns the clamped output.
 *
 * ref  : reference value
 * meas : measured feedback value
 */
float FOC_IP_Update(FOC_IP_t *ip, float ref, float meas);

#endif /* FOC_IP_H */
