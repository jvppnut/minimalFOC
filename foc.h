#ifndef FOC_H
#define FOC_H

#include "core/math/foc_pid.h"
#include "motor/foc_motor.h"

/* -------------------------------------------------------------------------
 * Module-level PID controller instances.
 *
 * Defined in foc.c; exposed here so the application can configure gains
 * directly via FOC_PID_Init() / FOC_PID_SetGains() before calling FOC_Init().
 *
 * Single-motor assumption: one set of controllers per translation unit.
 * ------------------------------------------------------------------------- */
extern FOC_PID_t foc_pid_id;
extern FOC_PID_t foc_pid_iq;
extern FOC_PID_t foc_pid_speed;
extern FOC_PID_t foc_pid_pos;

/* Zero all integrator state. Call once at startup after configuring gains. */
void FOC_Init(void);

/* Reset all PID integrators without touching gains or limits.
   Call on fault recovery or control mode switch. */
void FOC_Reset(void);

/*
 * Execute one FOC control step.
 *
 * Reads from motor->state (populated by HAL before the call).
 * Writes to motor->out  (consumed by HAL after the call).
 *
 * Contains no HAL calls — safe to unit-test on host.
 */
void FOC_Step(FOC_Motor_t *motor);

/*
 * Electrical angle offset calibration routine.
 *
 * Called once at startup before entering normal operation. Drives the motor
 * to a known voltage vector (FOC_MODE_VOLTAGE), waits for the rotor to
 * settle, then reads back theta_mech to compute and store
 * motor->hw.theta_elec_offset.
 *
 * Implementation pending — stub only.
 */
void FOC_Calibrate(FOC_Motor_t *motor);

#endif /* FOC_H */
