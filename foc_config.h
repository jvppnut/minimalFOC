#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* ==========================================================================
 * foc_config.h — compile-time configuration for the minimalFOC library.
 *
 * All user-tunable parameters live here.  When porting to a new motor or
 * hardware platform, this is the only file that should need to change.
 *
 * On the MCU these values are used to populate the FOC_Motor_t struct at
 * startup.  In the simulator they serve as a shared reference so that the
 * C library and the Python physics model always agree on motor parameters.
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * Control loop timing
 * -------------------------------------------------------------------------- */
#define FOC_TS_HW           50e-6f      /* Control loop period            (s)  — 20 kHz  */
#define FOC_TS_SIM           5e-6f      /* Simulator physics sub-step     (s)  — 200 kHz
                                           Not used on the MCU.                           */

/* --------------------------------------------------------------------------
 * Motor electrical parameters
 * -------------------------------------------------------------------------- */
#define FOC_MOTOR_RS            0.5f    /* Stator resistance              (Ohm)           */
#define FOC_MOTOR_LD            1e-3f   /* d-axis inductance              (H)             */
#define FOC_MOTOR_LQ            1e-3f   /* q-axis inductance              (H)             */
#define FOC_MOTOR_LAMBDA_PM     0.01f   /* Permanent-magnet flux linkage  (Wb)            */
#define FOC_MOTOR_POLE_PAIRS    7       /* Number of pole pairs                           */

/* --------------------------------------------------------------------------
 * Motor mechanical parameters
 * -------------------------------------------------------------------------- */
#define FOC_MOTOR_J             1e-4f   /* Rotor inertia                  (kg·m²)         */
#define FOC_MOTOR_DM            1e-4f   /* Viscous friction coefficient   (N·m·s/rad)     */
#define FOC_MOTOR_RATED_CURRENT 5.0f    /* Peak rated phase current       (A)             */
#define FOC_MOTOR_RATED_SPEED   200.0f  /* Rated mechanical speed         (rad/s)         */

/* --------------------------------------------------------------------------
 * Inverter / hardware
 * -------------------------------------------------------------------------- */
#define FOC_V_BUS_NOMINAL       24.0f   /* Nominal DC bus voltage         (V)             */
#define FOC_DEAD_TIME           500e-9f /* PWM dead time                  (s)             */
#define FOC_DUTY_MAX            0.9f    /* Maximum PWM duty cycle         [0, 1]
                                           Leaves 10 % minimum low-side on-time for
                                           low-side shunt current sensing.                */
#define FOC_PWM_ACTIVE_LOW      0       /* 1 if MCU PWM output is active-low              */

/* --------------------------------------------------------------------------
 * Calibration defaults
 * -------------------------------------------------------------------------- */
#define FOC_CAL_V_D             2.0f    /* d-axis alignment voltage       (V)
                                           Typical: 10–20 % of v_bus.                    */
#define FOC_CAL_SETTLE_TIME     0.5f    /* Rotor settle time during cal   (s)             */

/* --------------------------------------------------------------------------
 * Estimator
 * -------------------------------------------------------------------------- */
/* Velocity LPF coefficient: alpha = exp(-2π × cutoff_hz × Ts_hw).
   Recompute when changing cutoff or Ts.
   Current setting: cutoff = 200 Hz, Ts = 50 µs → exp(-2π×200×50e-6) ≈ 0.9394 */
#define FOC_ESTIMATOR_LPF_ALPHA 0.9394f

#endif /* FOC_CONFIG_H */
