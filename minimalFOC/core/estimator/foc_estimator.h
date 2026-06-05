#ifndef FOC_ESTIMATOR_H
#define FOC_ESTIMATOR_H

#include "motor/foc_motor.h"

/* -------------------------------------------------------------------------
 * Single-instance velocity estimator.
 *
 * Reads theta_mech_st from motor->state each cycle (direction-corrected
 * single-turn angle), unwraps it to a continuous multi-turn angle, and
 * estimates mechanical velocity via backward difference + first-order IIR LPF.
 *
 * Writes:
 *   motor->state.theta_mech — unwrapped multi-turn angle (rad), correct sign
 *   motor->state.omega_mech — LPF-filtered velocity (rad/s), correct sign
 *
 * Called from inside FOC_Step(), after theta_mech_st is computed from
 * theta_mech_raw. No separate sign correction needed after this call.
 *
 * Call order each control cycle (handled by FOC_Step internally):
 *   1. HAL writes motor->state.theta_mech_raw  (single-turn [0, 2π))
 *   2. FOC_Step() computes theta_mech_st (direction-corrected)
 *   3. FOC_Estimator_Update() unwraps and differentiates theta_mech_st
 *
 * Single-motor design: state is held in static variables — not suitable
 * for multi-instance use.
 * ------------------------------------------------------------------------- */

/* Configure LPF coefficient and sampling period.
   lpf_alpha = exp(-2π × cutoff_hz × Ts); precompute and store in foc_config.h.
   Resets all internal state. Call once at startup after HW config is set. */
void FOC_Estimator_Init(float lpf_alpha, float Ts);

/* Reset internal state (multi-turn angle, velocity, first-call flag).
   Does not change LPF configuration set by FOC_Estimator_Init(). */
void FOC_Estimator_Reset(void);

/* Run one estimation step. */
void FOC_Estimator_Update(FOC_Motor_t *motor);

#endif /* FOC_ESTIMATOR_H */
