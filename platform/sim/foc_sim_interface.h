#ifndef FOC_SIM_INTERFACE_H
#define FOC_SIM_INTERFACE_H

#include <stdint.h>

/* Initialize FOC and reset all PID state. Call once before anything else. */
void FOC_Sim_Init(void);

/* Motor physical parameters. */
void FOC_Sim_SetMotorParams(float Rs, float Ld, float Lq, float lambda_pm,
                             uint8_t pole_pairs, float J, float Dm,
                             float rated_current, float rated_speed);

/* Inverter and calibration configuration. */
void FOC_Sim_SetHWConfig(float Ts, float dead_time, float v_bus_nominal,
                          float duty_max, uint8_t pwm_active_low,
                          float theta_elec_offset);

/* PID gains for each axis (discrete-scaled, caller's choice of method). */
void FOC_Sim_SetPIDGains_Id   (float Kp, float Ki, float Kd, float out_min, float out_max);
void FOC_Sim_SetPIDGains_Iq   (float Kp, float Ki, float Kd, float out_min, float out_max);
void FOC_Sim_SetPIDGains_Speed(float Kp, float Ki, float Kd, float out_min, float out_max);
void FOC_Sim_SetPIDGains_Pos  (float Kp, float Ki, float Kd, float out_min, float out_max);

/* Control mode (FOC_MODE_* constant from foc_motor.h). */
void FOC_Sim_SetMode(uint8_t mode);

/* Set all reference fields at once. Unused fields for the active mode are ignored. */
void FOC_Sim_SetRef(float v_d_ref, float v_q_ref, float i_d_ref,
                    float i_q_ref, float omega_ref, float theta_ref);

/* Reset all PID integrators without changing gains or limits.
   Does not reset estimator state (position and velocity are preserved). */
void FOC_Sim_Reset(void);

/* Reset estimator state (multi-turn angle, velocity, first-call flag).
   Does not affect PID state or LPF configuration. */
void FOC_Sim_EstimatorReset(void);

/* Configure the velocity estimator. Must be called after FOC_Sim_SetHWConfig().
   lpf_alpha = exp(-2π × cutoff_hz × Ts); precompute this value in the caller. */
void FOC_Sim_EstimatorInit(float lpf_alpha);

/* Arm electrical angle offset calibration. Switches mode to FOC_MODE_CALIBRATE
   and stores calibration parameters. Normal FOC_Sim_Step() calls continue
   unchanged; FOC_Step() handles the settle countdown internally. */
void FOC_Sim_Calibrate(float v_cal, float settle_time_s);

/* Read the current control mode (FOC_MODE_* constant from foc_motor.h).
   Poll this to detect when FOC_MODE_CALIBRATE has completed. */
uint8_t FOC_Sim_GetMode(void);

/*
 * Run one FOC step.
 * The estimator runs first, computing theta_mech and omega_mech from
 * theta_mech_raw before passing control to FOC_Step().
 *
 * Inputs : phase currents (A), single-turn encoder angle (rad),
 *          DC bus voltage (V).
 * Outputs: PWM duty cycles [0, 1] for all three phases.
 */
void FOC_Sim_Step(float i_u, float i_v, float i_w,
                  float theta_mech_raw, float v_bus,
                  float *duty_u, float *duty_v, float *duty_w);

/*
 * Read FOC internal signals after a step for logging and plotting.
 * Any pointer may be NULL to skip that field.
 */
void FOC_Sim_GetInternals(float *theta_elec,
                           float *i_alpha, float *i_beta,
                           float *i_d,     float *i_q,
                           float *v_d,     float *v_q,
                           float *v_alpha, float *v_beta);

/*
 * Read estimator output after a step.
 * Any pointer may be NULL to skip that field.
 */
void FOC_Sim_GetEstimatorState(float *theta_mech, float *omega_mech);

#endif /* FOC_SIM_INTERFACE_H */
