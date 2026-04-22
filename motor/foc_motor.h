#ifndef FOC_MOTOR_H
#define FOC_MOTOR_H

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Control mode
 * -------------------------------------------------------------------------- */
typedef uint8_t FOC_CtrlMode_t;

#define FOC_MODE_TORQUE   ((FOC_CtrlMode_t)0)  /* Inner current loop only — i_q_ref drives   */
#define FOC_MODE_VELOCITY ((FOC_CtrlMode_t)1)  /* Speed loop active       — omega_ref drives */
#define FOC_MODE_POSITION ((FOC_CtrlMode_t)2)  /* Position loop active    — theta_ref drives */

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
 * Set once at initialisation. Never written during operation.
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

    /* Angle offsets applied by the FOC before use:
     *   theta_elec = (theta_mech - theta_mech_offset) * pole_pairs
     *                + theta_elec_offset
     * theta_mech in FOC_MotorState_t always stores the raw sensor value.  */
    float  theta_mech_offset; /* Mechanical zero offset (robot link) (rad) */
    float  theta_elec_offset; /* Electrical zero offset (encoder cal) (rad)*/
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
    float  theta_mech;  /* Raw encoder angle              (rad)            */
    float  theta_elec;  /* Electrical angle, offset-applied (rad)          */
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
