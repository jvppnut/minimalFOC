#ifndef FOC_H
#define FOC_H

#include "core/math/foc_pid.h"
#include "core/math/foc_ip.h"
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

/* I-P alternates for the current loop — same Kp/Ki, no reference-path zero.
 * Kept initialised alongside foc_pid_id/iq at all times; which one actually
 * drives v_d/v_q is selected in FOC_CurrentCtrlComputation() (foc.c) by
 * commenting/uncommenting the two-line block there. Used to check whether an
 * observed step-response mismatch comes from the PI zero or from the poles
 * themselves. */
extern FOC_IP_t foc_ip_id;
extern FOC_IP_t foc_ip_iq;

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
 * Position error is absolute (unwrapped) — theta_ref is tracked as a
 * genuine multi-turn target, not a shortest-path angle mod 2*pi. Needed
 * for correct output-side tracking through a reduction ratio.
 *
 * ref.theta_ref is tracked as given — this function has no notion of where
 * the reference comes from or whether it's been shaped into a smooth
 * trajectory. Reference shaping (e.g. trapezoidal profiling to avoid large
 * current transients from a raw step) is the caller's responsibility; see
 * core/math/foc_trapgen.h for a reusable profile generator intended to be
 * driven by the application layer, one level above this library.
 *
 * Reads  : state.theta_mech, omega_mech; ref.theta_ref
 * Writes : ref.i_q_ref
 */
void FOC_PositionCtrlComputation(FOC_Motor_t *motor);

/*
 * Arm the electrical angle offset calibration.
 *
 * Sets r->mode to FOC_MODE_CALIBRATE and stores the calibration parameters in
 * motor->hw.  The caller continues its normal FOC_Step() loop unchanged.
 * FOC_Step() forces theta_elec = 0, applies v_d = v_cal / v_q = 0 for
 * settle_time_s seconds, then reads theta_mech, writes hw->theta_mech_offset
 * and hw->theta_elec_offset, and switches r->mode back to FOC_MODE_VOLTAGE.
 *
 * v_cal        — d-axis alignment voltage (V); 10–20 % of v_bus is typical.
 * settle_time_s — time to hold before reading the encoder (s); must be long
 *                 enough for the rotor to damp out (typically 0.5–2 s).
 */
void FOC_Calibrate(FOC_Motor_t *motor, float v_cal, float settle_time_s);

#endif /* FOC_H */
