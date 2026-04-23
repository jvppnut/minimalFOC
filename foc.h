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
 *
 * foc_pid_id    — d-axis current PI   (Ki=0 for P-only, Kd=0 always)
 * foc_pid_iq    — q-axis current PI   (same)
 * foc_pid_speed — speed PI            (used in FOC_MODE_VELOCITY)
 * foc_pid_pos   — position PD         (Ki=0; Kd provides damping via omega_mech)
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

/* -------------------------------------------------------------------------
 * Sub-steps called by FOC_Step().  Also exposed so they can be unit-tested
 * or invoked directly from specialised application code.
 *
 * All three functions read motor->state and motor->params; they write into
 * motor->ref and/or motor->out as described below.
 * ------------------------------------------------------------------------- */

/*
 * Inner current loop (all current-controlled modes).
 *
 * Reads  : state.i_d, i_q, omega_mech, v_bus; params.Ld, Lq, lambda_pm, pole_pairs
 *          ref.i_d_ref, i_q_ref
 * Writes : out.v_d, out.v_q
 *
 * Runs PI feedback on each axis and adds dq cross-coupling + back-EMF
 * feedforward so each axis behaves as a decoupled first-order R-L plant.
 * The total output is clamped to ±(v_bus / sqrt(3)).
 */
void FOC_CurrentCtrlComputation(FOC_Motor_t *motor);

/*
 * Outer speed loop (FOC_MODE_VELOCITY).
 *
 * Reads  : state.omega_mech; ref.omega_ref
 * Writes : ref.i_q_ref
 */
void FOC_VelocityCtrlComputation(FOC_Motor_t *motor);

/*
 * Outer position loop (FOC_MODE_POSITION).
 *
 * PD controller — no intermediate velocity loop.
 * The D term is the measured omega_mech (negative sign applied internally),
 * which damps the approach without a separate speed setpoint.
 * Position error is wrapped to (−π, π] for shortest-path tracking.
 *
 * Reads  : state.theta_mech, omega_mech; ref.theta_ref
 * Writes : ref.i_q_ref
 */
void FOC_PositionCtrlComputation(FOC_Motor_t *motor);

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
