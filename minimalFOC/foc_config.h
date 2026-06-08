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
 * Motor electrical parameters — CubeMars AK60-6 V1.1 (KV80)
 *
 * Delta winding. Spec gives line-to-line values; Y-equivalents used here:
 *   Rs_LL = 202 mΩ  →  Rs_Y  = Rs_LL / 2  = 101 mΩ
 *   L_LL  = 138 µH  →  Ls_Y  = L_LL  / 2  = 69 µH
 *   lambda_pm derived from Kt = 0.078 Nm/A:  λ = 2·Kt / (3·p) = 3.71 mWb
 * Verify Rs and Ls with a locked-rotor impedance measurement before tuning PIDs.
 * -------------------------------------------------------------------------- */
// #define FOC_MOTOR_RS         0.101f  /* spec: Rs_LL=202mΩ → Y-equiv=101mΩ             */
#define FOC_MOTOR_RS            0.35f   /* measured: locked-rotor vd&vq tests (incl. cables) (Ohm) */
#define FOC_MOTOR_LD            69e-6f  /* d-axis inductance   Y-equiv    (H)             */
#define FOC_MOTOR_LQ            69e-6f  /* q-axis inductance   Y-equiv    (H)             */
// #define FOC_MOTOR_LAMBDA_PM  0.00371f/* spec: from Kt=0.078 Nm/A peak                 */
#define FOC_MOTOR_LAMBDA_PM     0.0055f /* measured: back-EMF vs speed at 1.5V open-loop  (Wb) */
#define FOC_MOTOR_POLE_PAIRS    14      /* Number of pole pairs                           */
// #define FOC_MOTOR_POLE_PAIRS    1       /* Number of pole pairs  (1 for only testing duty output respect to angle)                        */
/* --------------------------------------------------------------------------
 * Motor mechanical parameters — CubeMars AK60-6 V1.1 (KV80)
 *
 * Rated speed is motor-shaft (pre-gearbox): 420 rpm × 6 × 2π/60 = 264 rad/s
 * Rotor inertia (spec): 243.5 g·cm² = 2.435e-5 kg·m² — verify motor vs output side.
 * Dm not specified; 1e-4 is a placeholder — identify from deceleration test.
 * -------------------------------------------------------------------------- */
#define FOC_MOTOR_J             2.435e-5f/* Rotor inertia                 (kg·m²)         */
#define FOC_MOTOR_DM            1e-4f   /* Viscous friction (estimated)   (N·m·s/rad)     */
#define FOC_MOTOR_RATED_CURRENT 6.5f    /* Rated phase current            (A)  peak=13.1A */
#define FOC_MOTOR_RATED_SPEED   264.0f  /* Motor shaft rated speed        (rad/s) out=44  */

/* --------------------------------------------------------------------------
 * Inverter / hardware
 * -------------------------------------------------------------------------- */
#define FOC_V_BUS_NOMINAL       24.0f   /* Nominal DC bus voltage         (V)             */
#define FOC_DEAD_TIME           500e-9f /* PWM dead time                  (s)             */
#define FOC_DUTY_MAX            0.9f    /* Maximum PWM duty cycle         [0, 1]
                                           Leaves 10 % minimum low-side on-time for
                                           low-side shunt current sensing.                */
#define FOC_PWM_ACTIVE_LOW      1       /* 1 if MCU PWM output is active-low              */

/* Current sensing — must match PCB shunt and DRV8323 CSA_GAIN register field.
 * ADC_CSA_GAIN_VV must be consistent with FOC_DRV8323_CSA_GAIN (3 → 40 V/V). */
#define FOC_ADC_VREF            3.3f    /* ADC reference voltage          (V)             */
#define FOC_ADC_BITS            12      /* ADC resolution                 (bits)          */
#define FOC_ADC_RSHUNT          3e-3f   /* Phase current shunt resistance (Ohm)           */
#define FOC_ADC_CSA_GAIN_VV     40.0f   /* CSA amplifier gain — CSA_GAIN=3 → 40 V/V      */

/* --------------------------------------------------------------------------
 * Calibration defaults
 * -------------------------------------------------------------------------- */
#define FOC_CAL_V_D             1.0f    /* d-axis alignment voltage       (V)
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
 * Hardware: TPH1R204PL FETs (40V, 150A, RDS(on)=1.24mΩ@50A, Qg=74nC@10V)
 *           3mΩ 2W precision shunt resistors, 24V bus, 5A rated motor current
 *
 * Field encoding reference (verify against datasheet):
 *
 *   PWM_MODE  [0-3]: 0=6x   1=3x   2=1x   3=independent
 *   IDRIVEP   [0-F]: 0=10mA  4=120mA  8=260mA  F=1000mA  (source, HS and LS)
 *   IDRIVEN   [0-F]: 0=20mA  4=240mA  8=520mA  F=2000mA  (sink,   HS and LS)
 *   TDRIVE    [0-3]: 0=500ns  1=1000ns  2=2000ns  3=4000ns
 *   TRETRY    [0-1]: 0=4ms  1=50us
 *   DEAD_TIME [0-3]: 0=100ns  1=200ns  2=400ns  3=800ns
 *   OCP_MODE  [0-3]: 0=latch  1=retry  2=report only  3=disabled
 *   OCP_DEG   [0-3]: 0=2us  1=4us  2=6us  3=8us
 *   VDS_LVL   [0-F]: 0=0.06V  1=0.13V  2=0.20V  3=0.26V  4=0.31V  5=0.45V  6=0.53V  7=0.60V
 *                    8=0.68V  9=0.75V  A=0.94V  B=1.13V  C=1.30V  D=1.50V  E=1.70V  F=1.88V
 *   CSA_GAIN  [0-3]: 0=5V/V  1=10V/V  2=20V/V  3=40V/V
 *   VREF_DIV  [0-1]: 0=VREF  1=VREF/2 (use 1 for bidirectional sensing)
 *   SEN_LVL   [0-7]: CSA OCP trip in 0.25V steps; trip current = val*0.25 / (Rshunt*gain)
 *
 * Design decisions:
 *
 *   Gate drive (IDRIVEP=8/260mA, IDRIVEN=8/520mA):
 *     Sized from Qg=74nC. Turn-on ~285ns (74nC/260mA), turn-off ~142ns (74nC/520mA).
 *     Asymmetric sink>source intentional: faster turn-off reduces shoot-through risk.
 *     Conservative slew rate keeps switching EMI low at 20kHz.
 *
 *   TDRIVE=1 (1000ns):
 *     Peak drive window must cover gate charge time. 1000ns > 285ns turn-on with margin.
 *
 *   DEAD_TIME=2 (400ns):
 *     Must exceed turn-on time (285ns). 400ns gives ~115ns margin. Increase if
 *     cross-conduction observed on first bring-up.
 *
 *   VDS_LVL=2 (0.20V) — hard short protection only:
 *     At 5A rated, VDS_on = 5A x 1.24mΩ = 6mV (25°C), ~19mV (150°C).
 *     0.20V threshold trips at ~161A (25°C) or ~54A (150°C).
 *     VDS-OCP is not the primary overcurrent limit here; use SEN_LVL for that.
 *
 *   CSA_GAIN=3 (40V/V) + VREF_DIV=1 (VREF/2 midpoint):
 *     Vshunt at 5A rated = 5A x 3mΩ = 15mV → CSA output = 600mV from midpoint.
 *     Bidirectional range: ±(VREF/2) / (Rshunt x gain) = ±1.65V / 0.12 = ±13.75A.
 *     ADC resolution at 12-bit / 3.3V: ~6.7mA/count. Good for 5A rated application.
 *     Use lower gain (e.g. 20V/V) if motor rated current exceeds ~13A.
 *
 *   SEN_LVL=3 (~8.3A trip):
 *     Threshold = 3 x 0.25V = 0.75V → trip at 0.75V / 0.12V/A = 6.25A above zero.
 *     Provides ~1.25x rated current headroom before hardware OCP triggers.
 *     Adjust proportionally if rated current changes: SEN_LVL = ceil(1.25 x I_rated x 0.12 / 0.25)
 *
 * Resulting SPI register values (cross-check against logic analyzer on first bring-up):
 *
 *   0x02  Driver Control  = 0x0A0  = 000_1010_0000b
 *           b[7]   = 1        OTW_REP (warn on nFAULT)
 *           b[6:5] = 01       PWM_MODE = 3x
 *
 *   0x03  Gate Drive HS   = 0x388  = 011_1000_1000b
 *           b[10:8] = 011     LOCK field (power-on default, not written by init)
 *           b[7:4]  = 1000    IDRIVEP_HS = 8 (~260 mA source)
 *           b[3:0]  = 1000    IDRIVEN_HS = 8 (~520 mA sink)
 *
 *   0x04  Gate Drive LS   = 0x588  = 101_1000_1000b
 *           b[10]  = 1        CBC = cycle-by-cycle (default, active in retry OCP mode)
 *           b[9:8] = 01       TDRIVE = 1000 ns
 *           b[7:4] = 1000     IDRIVEP_LS = 8 (~260 mA source)
 *           b[3:0] = 1000     IDRIVEN_LS = 8 (~520 mA sink)
 *
 *   0x05  OCP Control     = 0x212  = 010_0001_0010b
 *           b[10]  = 0        TRETRY = 4 ms
 *           b[9:8] = 10       DEAD_TIME = 400 ns
 *           b[7:6] = 00       OCP_MODE = latch
 *           b[5:4] = 01       OCP_DEG = 4 us
 *           b[3:0] = 0010     VDS_LVL = 0.20 V
 *
 *
 *--> WRONG!!!
 *   0x06  CSA Control     = 0x2CC  = 010_1100_1100b
 *           b[9]   = 1        VREF_DIV = VREF/2 (bidirectional midpoint)
 *           b[7:6] = 11       CSA_GAIN = 40 V/V
 *           b[4:2] = 011      SEN_LVL = 3 (~6.25 A trip)
 *
 *
 * -->CORRECT
 *  *   0x06  CSA Control     = 0x2C3  = 010_1100_0011b
 *           b[9]   = 1        VREF_DIV = VREF/2 (bidirectional midpoint)
 *           b[7:6] = 11       CSA_GAIN = 40 V/V
 *           b[1:0] = 11      SEN_LVL = 3 (~6.25 A trip)
 * -------------------------------------------------------------------------- */

/* Driver Control (0x02) */
#define FOC_DRV8323_PWM_MODE        1u  /* 0=6x  1=3x  2=1x  3=independent   */
#define FOC_DRV8323_DIS_CPUV        0u
#define FOC_DRV8323_DIS_GDF         0u
#define FOC_DRV8323_OTW_REP         1u

/* Gate Drive HS (0x03) */
#define FOC_DRV8323_IDRIVEP_HS      8u              /* ~260 mA source       */
#define FOC_DRV8323_IDRIVEN_HS      8u              /* ~520 mA sink         */

/* Gate Drive LS (0x04) */
#define FOC_DRV8323_CBC             1u  /* 1=cycle-by-cycle (default); only active in OCP_MODE=retry */
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
#define FOC_DRV8323_CSA_GAIN        3u  /* 0=5V/V  1=10V/V  2=20V/V  3=40V/V */
#define FOC_DRV8323_VREF_DIV        1u              /* VREF/2 — bidirectional sensing */
#define FOC_DRV8323_LS_REF          0u
#define FOC_DRV8323_CSA_FET         0u              /* sense across shunt   */
#define FOC_DRV8323_DIS_SEN         0u
#define FOC_DRV8323_SEN_LVL         3u              /* ~8.3 A trip @ 3mΩ / 40V/V    */

#endif /* FOC_CONFIG_H */
