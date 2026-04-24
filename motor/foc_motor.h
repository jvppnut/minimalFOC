#ifndef FOC_MOTOR_H
#define FOC_MOTOR_H

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Control mode
 * -------------------------------------------------------------------------- */
typedef uint8_t FOC_CtrlMode_t;

#define FOC_MODE_VOLTAGE   ((FOC_CtrlMode_t)0)  /* Direct dq voltage       — v_d_ref/v_q_ref  */
#define FOC_MODE_TORQUE    ((FOC_CtrlMode_t)1)  /* Inner current loop only — i_q_ref drives   */
#define FOC_MODE_VELOCITY  ((FOC_CtrlMode_t)2)  /* Speed loop active       — omega_ref drives */
#define FOC_MODE_POSITION  ((FOC_CtrlMode_t)3)  /* Position loop active    — theta_ref drives */
#define FOC_MODE_CALIBRATE ((FOC_CtrlMode_t)4)  /* Electrical angle offset calibration        */

/* --------------------------------------------------------------------------
 * Physical motor constants
 * Set once at initialisation. Never written during operation.
 * -------------------------------------------------------------------------- */
typedef struct {
    float    Rs;            /* Stator resistance              (Ohm)        */
    float    Ld;            /* d-axis inductance              (H)          */
    float    Lq;            /* q-axis inductance              (H)          */
    float    lambda_pm;     /* Permanent-magnet flux linkage  (Wb)         */
    uint8_t  pole_pairs;    /* Number of pole pairs                        */
    float    J;             /* Rotor inertia                  (kg*m^2)     */
    float    Dm;            /* Viscous friction coefficient   (N*m*s/rad)  */
    float    rated_current; /* Peak rated current             (A)          */
    float    rated_speed;   /* Rated mechanical speed         (rad/s)      */
} FOC_MotorParams_t;

/* --------------------------------------------------------------------------
 * Inverter and system calibration
 * Most fields are set once at initialisation.  The cal_* fields are written
 * by FOC_Calibrate() and consumed / cleared by FOC_Step().
 * -------------------------------------------------------------------------- */
typedef struct {
    float  Ts;                /* Control loop sampling period   (s)        */
    float  dead_time;         /* PWM dead time                  (s)        */
    float  v_bus_nominal;     /* Nominal DC bus voltage         (V)        */
    float    duty_max;        /* Maximum PWM duty cycle         [0.0, 1.0]
                                 Low-side shunt sensing requires the low-side
                                 MOSFET to conduct for a minimum window each
                                 period.  Set to (1 - min_low_side_on_fraction),
                                 e.g. 0.9 for a 10 % minimum on-time.        */
    uint8_t  pwm_active_low; /* Non-zero if the MCU PWM output is active-low.
                                 Duties are inverted (1 - duty) before output. */

    /* Electrical angle (electrical radians), derived from the single-turn encoder:
     *   theta_elec = theta_mech_raw * pole_pairs + theta_elec_offset
     * theta_elec_offset is written by FOC_Calibrate() and is power-cycle persistent
     * (independent of the multi-turn software counter). */
    float  theta_elec_offset; /* Electrical zero offset, encoder cal  (rad) */

    /* Mechanical zero for joint-level position control.
     * Not used in the electrical angle path.
     * Set once at robot homing; consumed by FOC_PositionCtrlComputation(). */
    float  theta_mech_offset; /* Joint mechanical zero offset         (rad) */

    /* Non-zero if phase V and W are wired in reverse order (UWV instead of UVW).
     * Corrected by FOC_Step(): negates i_beta after Clarke, swaps duty_v/duty_w
     * after SVPWM.  Set by the direction-check calibration phase (future) or
     * manually if the wiring is known. */
    uint8_t  phase_reversed;

    /* Calibration state — written by FOC_Calibrate(), consumed by FOC_Step(). */
    uint32_t cal_steps_remaining; /* Settle countdown; 0 = calibration complete */
    float    cal_v_d;             /* d-axis voltage applied during alignment (V) */
} FOC_HWConfig_t;

/* --------------------------------------------------------------------------
 * Sensor readings and FOC-computed intermediates
 *
 * Written by HAL / simulator each control cycle:
 *   i_u, i_v, i_w, theta_mech, omega_mech, v_bus, temp
 *
 * Written by FOC core (stored for telemetry / debugging):
 *   theta_elec, i_alpha, i_beta, i_d, i_q
 * -------------------------------------------------------------------------- */
typedef struct {
    float  theta_mech_raw; /* Single-turn encoder angle [0, 2π)  (rad)      */
    float  theta_mech;     /* Multi-turn mechanical angle        (rad)      */
    float  theta_elec;     /* Electrical angle, offset-applied  (rad)      */
    float  omega_mech;  /* Mechanical angular velocity    (rad/s)          */
    float  i_u;         /* Phase U current                (A)              */
    float  i_v;         /* Phase V current                (A)              */
    float  i_w;         /* Phase W current                (A)              */
    float  i_alpha;     /* Stationary-frame alpha current (A)              */
    float  i_beta;      /* Stationary-frame beta current  (A)              */
    float  i_d;         /* Rotating-frame d current       (A)              */
    float  i_q;         /* Rotating-frame q current       (A)              */
    float  v_bus;       /* Measured DC bus voltage        (V)              */
    float  temp;        /* Motor temperature              (deg C)          */
} FOC_MotorState_t;

/* --------------------------------------------------------------------------
 * Setpoints
 * Written by the outer control layer or the user application.
 * -------------------------------------------------------------------------- */
typedef struct {
    FOC_CtrlMode_t  mode;    /* Active control mode                        */
    float  v_d_ref;          /* d-axis voltage reference       (V)         */
    float  v_q_ref;          /* q-axis voltage reference       (V)         */
    float  i_d_ref;          /* d-axis current reference       (A)         */
    float  i_q_ref;          /* q-axis current reference       (A)         */
    float  omega_ref;        /* Speed reference                (rad/s)     */
    float  theta_ref;        /* Position reference             (rad)       */
} FOC_MotorRef_t;

/* --------------------------------------------------------------------------
 * Commands
 * Written by the FOC core each cycle. Consumed by HAL or simulator.
 *
 * STM32 consumes    : duty_u/v/w
 * Simulator consumes: v_u/v/w  (equivalent: v_x = (duty_x - 0.5) * v_bus)
 * -------------------------------------------------------------------------- */
typedef struct {
    float  v_d;      /* d-axis voltage command         (V)                 */
    float  v_q;      /* q-axis voltage command         (V)                 */
    float  v_alpha;  /* Alpha voltage after inv. Park  (V)                 */
    float  v_beta;   /* Beta  voltage after inv. Park  (V)                 */
    float  v_u;      /* Phase U voltage reference      (V)                 */
    float  v_v;      /* Phase V voltage reference      (V)                 */
    float  v_w;      /* Phase W voltage reference      (V)                 */
    float  duty_u;   /* Phase U PWM duty cycle         [0.0, 1.0]          */
    float  duty_v;   /* Phase V PWM duty cycle         [0.0, 1.0]          */
    float  duty_w;   /* Phase W PWM duty cycle         [0.0, 1.0]          */
} FOC_MotorOut_t;

/* --------------------------------------------------------------------------
 * Fault flags (bitmask)
 * -------------------------------------------------------------------------- */
#define FOC_FAULT_NONE          (0u)
#define FOC_FAULT_OVERCURRENT   (1u << 0)
#define FOC_FAULT_OVERVOLTAGE   (1u << 1)
#define FOC_FAULT_UNDERVOLTAGE  (1u << 2)
#define FOC_FAULT_OVERTEMP      (1u << 3)
#define FOC_FAULT_ENCODER       (1u << 4)
#define FOC_FAULT_WATCHDOG      (1u << 5)

/* --------------------------------------------------------------------------
 * Top-level motor handle
 * One instance per motor. All FOC functions take a pointer to this struct.
 * -------------------------------------------------------------------------- */
typedef struct {
    FOC_MotorParams_t  params;
    FOC_HWConfig_t     hw;
    FOC_MotorState_t   state;
    FOC_MotorRef_t     ref;
    FOC_MotorOut_t     out;
    uint32_t           fault;
} FOC_Motor_t;

#endif /* FOC_MOTOR_H */
