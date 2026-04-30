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

/* --------------------------------------------------------------------------
 * DRV8323R gate driver
 *
 * Gate drive current codes — peak drive current per datasheet table.
 * Approximate source (IDRIVEP): 0→10mA  4→120mA  8→260mA  15→1000mA  verify
 * Approximate sink   (IDRIVEN): 0→20mA  4→240mA  8→520mA  15→2000mA  verify
 *
 * dead_time [0-4]: 0→100ns 1→200ns 2→400ns 3→800ns 4→1000ns           verify
 * ocp_mode  [0-3]: 0→latch 1→retry 2→report only 3→disabled            verify
 * ocp_deg   [0-3]: 0→2µs 1→4µs 2→6µs 3→8µs                            verify
 * vds_lvl   [0-3]: 0→0.06V 1→0.13V 2→0.20V 3→0.26V                    verify
 * tdrive    [0-3]: 0→500ns 1→1000ns 2→2000ns 3→4000ns                  verify
 * tretry    [0-1]: 0→4ms 1→50µs                                         verify
 * sen_lvl   [0-7]: sense OCP threshold — see datasheet table            verify
 * -------------------------------------------------------------------------- */

/* Driver Control (0x02) */
#define FOC_DRV8323_PWM_MODE        DRV8323_PWM_3X  /* 3x PWM               */
#define FOC_DRV8323_DIS_CPUV        0u
#define FOC_DRV8323_DIS_GDF         0u
#define FOC_DRV8323_OTW_REP         1u

/* Gate Drive HS (0x03) */
#define FOC_DRV8323_IDRIVEP_HS      8u              /* ~260 mA source       */
#define FOC_DRV8323_IDRIVEN_HS      8u              /* ~520 mA sink         */

/* Gate Drive LS (0x04) */
#define FOC_DRV8323_IDRIVEP_LS      8u
#define FOC_DRV8323_IDRIVEN_LS      8u
#define FOC_DRV8323_TDRIVE          1u              /* 1000 ns              */

/* OCP Control (0x05) */
#define FOC_DRV8323_TRETRY          0u              /* 4 ms                 */
#define FOC_DRV8323_DEAD_TIME       2u              /* 400 ns               */
#define FOC_DRV8323_OCP_MODE        0u              /* latch                */
#define FOC_DRV8323_OCP_DEG         1u              /* 4 µs                 */
#define FOC_DRV8323_VDS_LVL         2u              /* 0.20 V               */

/* CSA Control (0x06) */
#define FOC_DRV8323_CSA_GAIN        DRV8323_CSA_GAIN_40X  /* 40 V/V         */
#define FOC_DRV8323_VREF_DIV        0u
#define FOC_DRV8323_LS_REF          0u
#define FOC_DRV8323_CSA_FET         0u              /* sense across shunt   */
#define FOC_DRV8323_DIS_SEN         0u
#define FOC_DRV8323_SEN_LVL         0u

#endif /* FOC_CONFIG_H */
