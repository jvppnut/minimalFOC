"""
FOC Simulator
=============
Python PMSM motor model coupled to the compiled C FOC library via ctypes.
Motor physics run in Python/NumPy; the FOC algorithm runs in the actual
compiled C FOC_Step().

Build the shared library first (from the project root):
    Windows : build_sim.bat
    Linux   : ./build_sim.sh
    macOS   : ./build_sim.sh
"""

import ctypes
import os
import platform
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_TWO_PI_3       = 2.0 * np.pi / 3.0
_SQRT3_OVER_2   = np.sqrt(3.0) / 2.0
_ONE_OVER_SQRT3 = 1.0 / np.sqrt(3.0)

# FOC mode constants — must stay in sync with foc_motor.h
FOC_MODE_VOLTAGE   = 0
FOC_MODE_TORQUE    = 1
FOC_MODE_VELOCITY  = 2
FOC_MODE_POSITION  = 3
FOC_MODE_CALIBRATE = 4


# ---------------------------------------------------------------------------
# Parameter containers
# ---------------------------------------------------------------------------

class MotorParams:
    def __init__(self, Rs, Ld, Lq, lambda_pm, pole_pairs,
                 J, Dm, rated_current, rated_speed):
        self.Rs            = float(Rs)
        self.Ld            = float(Ld)
        self.Lq            = float(Lq)
        self.lambda_pm     = float(lambda_pm)
        self.pole_pairs    = int(pole_pairs)
        self.J             = float(J)
        self.Dm            = float(Dm)
        self.rated_current = float(rated_current)
        self.rated_speed   = float(rated_speed)


class HWConfig:
    def __init__(self, Ts, dead_time, v_bus_nominal, duty_max,
                 pwm_active_low=False, theta_elec_offset=0.0):
        self.Ts                = float(Ts)
        self.dead_time         = float(dead_time)
        self.v_bus_nominal     = float(v_bus_nominal)
        self.duty_max          = float(duty_max)
        self.pwm_active_low    = bool(pwm_active_low)
        self.theta_elec_offset = float(theta_elec_offset)


class PIDGains:
    def __init__(self, Kp, Ki, Kd, out_min, out_max):
        self.Kp      = float(Kp)
        self.Ki      = float(Ki)
        self.Kd      = float(Kd)
        self.out_min = float(out_min)
        self.out_max = float(out_max)


class VirtualPositionSensor:
    """
    Simulated single-turn position sensor.

    Wraps the true multi-turn mechanical angle to [0, 2π) and injects an
    electrical angle offset to emulate encoder misalignment.  This matches
    what a real absolute encoder (e.g. AS5600) would report on an STM32.

    Parameters
    ----------
    pole_pairs       : int   — motor pole pairs
    elec_offset_rad  : float — electrical angle offset to inject (rad);
                               positive = encoder reads ahead of true angle
    """
    def __init__(self, pole_pairs: int, elec_offset_rad: float = 0.0):
        self.pole_pairs      = int(pole_pairs)
        self.elec_offset_rad = float(elec_offset_rad)

    def read(self, theta_mech_true: float) -> float:
        """Return single-turn encoder reading [0, 2π) with injected offset."""
        mech_offset = self.elec_offset_rad / self.pole_pairs
        return (theta_mech_true + mech_offset) % (2.0 * np.pi)


# ---------------------------------------------------------------------------
# PMSM motor model — UVW phase frame
# ---------------------------------------------------------------------------

class PMSMModel:
    """
    PMSM in the UVW (abc) phase frame.

    Assumptions
    -----------
    - Sinusoidal back-EMF distribution (surface-mounted PMSM or linearised model).
    - Zero mutual inductance between phases: per-phase inductance = Ld.
    - Balanced star winding with floating neutral: i_u + i_v + i_w = 0,
      so i_w = -i_u - i_v is derived and not integrated separately.
    - The neutral-point voltage is removed from the inverter output before
      applying to the phase inductances; this prevents the SVPWM
      zero-sequence component from appearing as a spurious drive voltage.

    State
    -----
    i_u, i_v (A) | omega_mech (rad/s) | theta_mech (rad, multi-turn)
    """

    def __init__(self, params: MotorParams):
        self.p          = params
        self.i_u        = 0.0
        self.i_v        = 0.0
        self.i_w        = 0.0
        self.omega_mech = 0.0
        self.theta_mech = 0.0

    def step(self, duty_u, duty_v, duty_w, v_bus, dt, load_torque=0.0):
        """Advance one timestep. Uses forward Euler integration."""
        p = self.p

        # Inverter output voltages (DC midpoint reference)
        v_u = (duty_u - 0.5) * v_bus
        v_v = (duty_v - 0.5) * v_bus
        v_w = (duty_w - 0.5) * v_bus

        # Remove floating neutral-point voltage so the motor only sees
        # differential (phase-to-neutral) components.
        v_n  = (v_u + v_v + v_w) / 3.0
        v_un = v_u - v_n
        v_vn = v_v - v_n
        # v_wn not needed — i_w is derived from the balanced condition

        # True electrical angle and speed (no calibration offsets)
        theta_e = self.theta_mech * p.pole_pairs
        omega_e = self.omega_mech * p.pole_pairs

        # Back-EMF: e_x = -lambda_pm * omega_e * sin(theta_e_x)
        e_u = -p.lambda_pm * omega_e * np.sin(theta_e)
        e_v = -p.lambda_pm * omega_e * np.sin(theta_e - _TWO_PI_3)
        e_w = -p.lambda_pm * omega_e * np.sin(theta_e + _TWO_PI_3)

        # Phase current ODEs  (L = Ld; zero mutual inductance assumed)
        d_iu = (v_un - p.Rs * self.i_u - e_u) / p.Ld
        d_iv = (v_vn - p.Rs * self.i_v - e_v) / p.Ld

        # Electromagnetic torque (uses pre-integration i_w)
        Te = -p.pole_pairs * p.lambda_pm * (
              self.i_u * np.sin(theta_e)
            + self.i_v * np.sin(theta_e - _TWO_PI_3)
            + self.i_w * np.sin(theta_e + _TWO_PI_3))

        # Mechanical dynamics
        d_omega = (Te - p.Dm * self.omega_mech - load_torque) / p.J
        d_theta = self.omega_mech

        # Forward Euler integration
        self.i_u        += d_iu    * dt
        self.i_v        += d_iv    * dt
        self.i_w         = -self.i_u - self.i_v   # enforce balanced constraint
        self.omega_mech += d_omega * dt
        self.theta_mech += d_theta * dt


# ---------------------------------------------------------------------------
# ctypes wrapper
# ---------------------------------------------------------------------------

class FOCLib:
    """Thin ctypes wrapper around the compiled C FOC shared library."""

    def __init__(self, lib_path: str):
        self._lib = ctypes.CDLL(lib_path)
        self._declare_types()

    def _declare_types(self):
        f  = self._lib
        cf = ctypes.c_float
        cu = ctypes.c_uint8
        cp = ctypes.POINTER(ctypes.c_float)

        f.FOC_Sim_Init.argtypes = []
        f.FOC_Sim_Init.restype  = None

        f.FOC_Sim_SetMotorParams.argtypes = [cf, cf, cf, cf, cu, cf, cf, cf, cf]
        f.FOC_Sim_SetMotorParams.restype  = None

        f.FOC_Sim_SetHWConfig.argtypes = [cf, cf, cf, cf, cu, cf]
        f.FOC_Sim_SetHWConfig.restype  = None

        for name in ('FOC_Sim_SetPIDGains_Id',    'FOC_Sim_SetPIDGains_Iq',
                     'FOC_Sim_SetPIDGains_Speed',  'FOC_Sim_SetPIDGains_Pos'):
            fn          = getattr(f, name)
            fn.argtypes = [cf] * 5
            fn.restype  = None

        f.FOC_Sim_SetMode.argtypes = [cu]
        f.FOC_Sim_SetMode.restype  = None

        f.FOC_Sim_SetRef.argtypes = [cf] * 6
        f.FOC_Sim_SetRef.restype  = None

        f.FOC_Sim_Reset.argtypes = []
        f.FOC_Sim_Reset.restype  = None

        f.FOC_Sim_Calibrate.argtypes = [cf, cf]
        f.FOC_Sim_Calibrate.restype  = None

        f.FOC_Sim_GetMode.argtypes = []
        f.FOC_Sim_GetMode.restype  = cu

        f.FOC_Sim_Step.argtypes = [cf, cf, cf, cf, cf, cf, cf, cp, cp, cp]
        f.FOC_Sim_Step.restype  = None

        f.FOC_Sim_GetInternals.argtypes = [cp] * 9
        f.FOC_Sim_GetInternals.restype  = None

    # --- public API --------------------------------------------------------

    def init(self, motor_params: MotorParams, hw: HWConfig):
        p = motor_params
        self._lib.FOC_Sim_SetMotorParams(
            p.Rs, p.Ld, p.Lq, p.lambda_pm, p.pole_pairs,
            p.J, p.Dm, p.rated_current, p.rated_speed)
        self._lib.FOC_Sim_SetHWConfig(
            hw.Ts, hw.dead_time, hw.v_bus_nominal, hw.duty_max,
            int(hw.pwm_active_low), hw.theta_elec_offset)
        self._lib.FOC_Sim_Init()

    def set_pid_gains(self, pid_id: PIDGains, pid_iq: PIDGains,
                      pid_speed: PIDGains, pid_pos: PIDGains):
        self._lib.FOC_Sim_SetPIDGains_Id(
            pid_id.Kp, pid_id.Ki, pid_id.Kd, pid_id.out_min, pid_id.out_max)
        self._lib.FOC_Sim_SetPIDGains_Iq(
            pid_iq.Kp, pid_iq.Ki, pid_iq.Kd, pid_iq.out_min, pid_iq.out_max)
        self._lib.FOC_Sim_SetPIDGains_Speed(
            pid_speed.Kp, pid_speed.Ki, pid_speed.Kd,
            pid_speed.out_min, pid_speed.out_max)
        self._lib.FOC_Sim_SetPIDGains_Pos(
            pid_pos.Kp, pid_pos.Ki, pid_pos.Kd,
            pid_pos.out_min, pid_pos.out_max)

    def set_mode(self, mode: int):
        self._lib.FOC_Sim_SetMode(mode)

    def set_ref(self, v_d_ref=0.0, v_q_ref=0.0, i_d_ref=0.0,
                i_q_ref=0.0, omega_ref=0.0, theta_ref=0.0):
        self._lib.FOC_Sim_SetRef(
            v_d_ref, v_q_ref, i_d_ref, i_q_ref, omega_ref, theta_ref)

    def reset(self):
        self._lib.FOC_Sim_Reset()

    def calibrate(self, v_cal: float, settle_time_s: float):
        self._lib.FOC_Sim_Calibrate(v_cal, settle_time_s)

    def get_mode(self) -> int:
        return int(self._lib.FOC_Sim_GetMode())

    def step(self, i_u, i_v, i_w, theta_mech_raw, theta_mech, omega_mech, v_bus):
        du, dv, dw = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
        self._lib.FOC_Sim_Step(
            i_u, i_v, i_w, theta_mech_raw, theta_mech, omega_mech, v_bus,
            ctypes.byref(du), ctypes.byref(dv), ctypes.byref(dw))
        return du.value, dv.value, dw.value

    def get_internals(self):
        vals = [ctypes.c_float() for _ in range(9)]
        self._lib.FOC_Sim_GetInternals(*[ctypes.byref(v) for v in vals])
        keys = ('theta_elec', 'i_alpha', 'i_beta',
                'i_d', 'i_q', 'v_d', 'v_q', 'v_alpha', 'v_beta')
        return {k: v.value for k, v in zip(keys, vals)}


# ---------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------

def _stack_samples(samples):
    """Convert a list of scalar sample dicts into a dict of numpy arrays."""
    return {k: np.array([s[k] for s in samples]) for k in samples[0]}


class FOCSimulator:
    """
    Closed-loop simulation: Python PMSM model (UVW frame) + compiled C FOC_Step().

    Two usage modes
    ---------------
    Step-based (multi-phase tests):
        sensor = VirtualPositionSensor(pole_pairs=7, elec_offset_rad=...)
        sim = FOCSimulator(motor, hw, LIB_PATH, sensor=sensor, Ts_sim=Ts_sim)
        sim.reset()
        sim.foc.set_mode(...)
        for each step:
            sim.foc.set_ref(...)
            sample = sim.step(load_torque=...)
        log = _stack_samples(collected_samples)

    Run-based (single-phase tests, backwards-compatible):
        log = sim.run(duration, mode, ref_fn=...)
    """

    def __init__(self, motor_params: MotorParams, hw: HWConfig, lib_path: str,
                 sensor: VirtualPositionSensor = None, Ts_sim: float = None):
        self.motor_params = motor_params
        self.hw           = hw
        self.Ts_sim       = float(Ts_sim) if Ts_sim is not None else hw.Ts
        self._n_sub       = max(1, round(hw.Ts / self.Ts_sim))
        # Default: ideal sensor with no offset.
        self.sensor       = sensor if sensor is not None \
                            else VirtualPositionSensor(motor_params.pole_pairs)
        self.foc          = FOCLib(lib_path)
        self.foc.init(motor_params, hw)

        # Mutable simulation state — reset() reinitialises these.
        self.motor    = PMSMModel(motor_params)
        self._duty_u  = 0.5
        self._duty_v  = 0.5
        self._duty_w  = 0.5
        self._t       = 0.0

    def set_pid_gains(self, pid_id, pid_iq, pid_speed, pid_pos):
        self.foc.set_pid_gains(pid_id, pid_iq, pid_speed, pid_pos)

    def reset(self):
        """Reset motor model, duty cycles, time counter, and PID integrators."""
        self.motor   = PMSMModel(self.motor_params)
        self._duty_u = 0.5
        self._duty_v = 0.5
        self._duty_w = 0.5
        self._t      = 0.0
        self.foc.reset()

    def step(self, load_torque=0.0):
        """
        Advance one Ts_hw control step.

        The active mode and references must be set on self.foc before calling.
        The position sensor (self.sensor) is read each step; swap it out between
        phases to change the injected offset without restarting the simulation.

        Returns a dict of scalar signal values for this step.
        """
        p   = self.motor_params
        hw  = self.hw

        # Sub-step motor physics at Ts_sim with previous cycle's duty cycles (ZOH).
        for _ in range(self._n_sub):
            self.motor.step(self._duty_u, self._duty_v, self._duty_w,
                            hw.v_bus_nominal, self.Ts_sim, load_torque)

        # Single-turn encoder reading [0, 2π) with any injected offset.
        theta_mech_raw = self.sensor.read(self.motor.theta_mech)

        # Run compiled C FOC step.
        self._duty_u, self._duty_v, self._duty_w = self.foc.step(
            self.motor.i_u, self.motor.i_v, self.motor.i_w,
            theta_mech_raw, self.motor.theta_mech, self.motor.omega_mech,
            hw.v_bus_nominal)

        ins = self.foc.get_internals()
        t   = self._t
        self._t += hw.Ts

        return {
            'time':              t,
            'theta_mech':        self.motor.theta_mech,
            'theta_mech_single': self.motor.theta_mech % (2.0 * np.pi),
            'theta_mech_raw':    theta_mech_raw,
            'omega_mech':        self.motor.omega_mech,
            'i_u':               self.motor.i_u,
            'i_v':               self.motor.i_v,
            'i_w':               self.motor.i_w,
            'i_d':               ins['i_d'],
            'i_q':               ins['i_q'],
            'v_d':               ins['v_d'],
            'v_q':               ins['v_q'],
            'duty_u':            self._duty_u,
            'duty_v':            self._duty_v,
            'duty_w':            self._duty_w,
            'theta_elec_true':   (self.motor.theta_mech * p.pole_pairs) % (2.0 * np.pi),
            'theta_elec_foc':    ins['theta_elec'] % (2.0 * np.pi),
        }

    def run(self, duration, mode, ref_kwargs=None, load_torque=0.0, ref_fn=None):
        """
        Run a closed-loop simulation from a clean state.

        The injected encoder offset (if any) comes from self.sensor.

        Parameters
        ----------
        duration   : float    — simulation duration (s)
        mode       : int      — FOC_MODE_* constant
        ref_kwargs : dict     — constant reference fields; ignored when ref_fn given
        load_torque: float    — constant mechanical load torque (N·m)
        ref_fn     : callable — ref_fn(t) -> dict; updates reference each step

        Returns
        -------
        dict of numpy arrays keyed by signal name.
        """
        n          = int(duration / self.hw.Ts)
        static_ref = ref_kwargs or {}

        self.reset()
        self.foc.set_mode(mode)
        if ref_fn is None:
            self.foc.set_ref(**static_ref)

        samples = []
        for _ in range(n):
            if ref_fn is not None:
                ref = ref_fn(self._t)
                self.foc.set_ref(**ref)
            else:
                ref = static_ref

            sample = self.step(load_torque=load_torque)
            sample['v_q_ref'] = ref.get('v_q_ref', 0.0)
            sample['i_q_ref'] = ref.get('i_q_ref', 0.0)
            samples.append(sample)

        return _stack_samples(samples)

    @staticmethod
    def plot(log, title='FOC Simulation'):
        t = log['time'] * 1e3   # ms

        fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
        fig.suptitle(title)

        axes[0].plot(t, log['omega_mech'])
        axes[0].set_ylabel('Speed (rad/s)')
        axes[0].grid(True)

        axes[1].plot(t, log['i_u'], label='i_u')
        axes[1].plot(t, log['i_v'], label='i_v')
        axes[1].plot(t, log['i_w'], label='i_w')
        axes[1].set_ylabel('Phase currents (A)')
        axes[1].legend()
        axes[1].grid(True)

        axes[2].plot(t, log['i_d'], label='i_d')
        axes[2].plot(t, log['i_q'], label='i_q')
        axes[2].set_ylabel('dq currents (A)')
        axes[2].legend()
        axes[2].grid(True)

        axes[3].plot(t, np.degrees(log['theta_elec_true']), label='true θ_e')
        axes[3].plot(t, np.degrees(log['theta_elec_foc']),  label='FOC θ_e', linestyle='--')
        axes[3].set_ylabel('Electrical angle (°)')
        axes[3].set_xlabel('Time (ms)')
        axes[3].legend()
        axes[3].grid(True)

        plt.tight_layout()
        plt.show()
