#ifndef FOC_PID_H
#define FOC_PID_H

#include "foc_math.h"

/*
 * Generic PID controller.
 *
 * Gains (Kp, Ki, Kd) are stored as-is. The caller is responsible for passing
 * gains that are already scaled for the chosen discretization method
 * (forward Euler, Tustin, etc.) and sample time.
 *
 * The derivative term is computed from a directly supplied measurement
 * derivative rather than numerical differentiation inside the controller.
 * In FOC this value is typically already available (e.g. omega for a
 * position loop, estimated acceleration for a speed loop), making internal
 * differentiation redundant and noisier.
 *
 * Set Kd = 0.0f for a PI controller (typical for current loops).
 * Set Ki = 0.0f for a PD controller.
 *
 * Anti-windup: the integrator is clamped to [out_min, out_max] each step.
 *
 * On-the-fly gain tuning (e.g. from a host PC):
 *   Call FOC_PID_SetGains() to update gains without resetting the integrator,
 *   so the controller continues bumplessly.
 */
typedef struct {
    float Kp;       /* Proportional gain                */
    float Ki;       /* Integral gain (discrete-scaled)  */
    float Kd;       /* Derivative gain (discrete-scaled)*/
    float out_min;  /* Output lower limit               */
    float out_max;  /* Output upper limit               */

    /* State — reset with FOC_PID_Reset() on enable or mode switch. */
    float integrator;
} FOC_PID_t;

/* Initialise gains and limits, and reset internal state. */
void  FOC_PID_Init(FOC_PID_t *pid, float Kp, float Ki, float Kd,
                   float out_min, float out_max);

/* Update gains without touching the integrator.
   Use for on-the-fly tuning so the controller continues bumplessly. */
void  FOC_PID_SetGains(FOC_PID_t *pid, float Kp, float Ki, float Kd);

/* Clear integrator without changing gains or limits. */
void  FOC_PID_Reset(FOC_PID_t *pid);

/*
 * Run one control step; returns the clamped output.
 *
 * error    : reference - measurement (computed by caller)
 * meas_dot : time derivative of the measurement (rad/s for position loop,
 *            rad/s^2 for speed loop). Pass 0.0f when Kd = 0.0f.
 */
float FOC_PID_Update(FOC_PID_t *pid, float error, float meas_dot);

#endif /* FOC_PID_H */
