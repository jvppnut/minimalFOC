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
FOC_MODE_VOLTAGE  = 0
FOC_MODE_TORQUE   = 1
FOC_MODE_VELOCITY = 2
FOC_MODE_POSITION = 3


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
                 pwm_active_low=False, theta_mech_offset=0.0, theta_elec_offset=0.0):
        self.Ts                = float(Ts)
        self.dead_time         = float(dead_time)
        self.v_bus_nominal     = float(v_bus_nominal)
        self.duty_max          = float(duty_max)
        self.pwm_active_low    = bool(pwm_active_low)
        self.theta_mech_offset = float(theta_mech_offset)
        self.theta_elec_offset = float(theta_elec_offset)


class PIDGains:
    def __init__(self, Kp, Ki, Kd, out_min, out_max):
        self.Kp      = float(Kp)
        self.Ki      = float(Ki)
        self.Kd      = float(Kd)
        self.out_min = float(out_min)
        self.out_max = float(out_max)


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
        self.omega_mech = 0.0
        self.theta_mech = 0.0

    @property
    def i_w(self):
        """Derived via balanced condition: i_w = -i_u - i_v."""
        return -self.i_u - self.i_v

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
        i_w  = self.i_w
        d_iu = (v_un - p.Rs * self.i_u - e_u) / p.Ld
        d_iv = (v_vn - p.Rs * self.i_v - e_v) / p.Ld
        # d_iw = -d_iu - d_iv  (implicit; consistent with balanced condition)

        # Electromagnetic torque
        Te = -p.pole_pairs * p.lambda_pm * (
              self.i_u * np.sin(theta_e)
            + self.i_v * np.sin(theta_e - _TWO_PI_3)
            + i_w      * np.sin(theta_e + _TWO_PI_3))

        # Mechanical dynamics
        d_omega = (Te - p.Dm * self.omega_mech - load_torque) / p.J
        d_theta = self.omega_mech

        # Forward Euler integration
        self.i_u        += d_iu    * dt
        self.i_v        += d_iv    * dt
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

        f.FOC_Sim_SetHWConfig.argtypes = [cf, cf, cf, cf, cu, cf, cf]
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

        f.FOC_Sim_Step.argtypes = [cf, cf, cf, cf, cf, cf, cp, cp, cp]
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
            int(hw.pwm_active_low), hw.theta_mech_offset, hw.theta_elec_offset)
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

    def step(self, i_u, i_v, i_w, theta_mech, omega_mech, v_bus):
        du, dv, dw = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
        self._lib.FOC_Sim_Step(
            i_u, i_v, i_w, theta_mech, omega_mech, v_bus,
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

class FOCSimulator:
    """
    Closed-loop simulation: Python PMSM model (UVW frame) + compiled C FOC_Step().
    """

    def __init__(self, motor_params: MotorParams, hw: HWConfig, lib_path: str):
        self.motor_params = motor_params
        self.hw           = hw
        self.foc          = FOCLib(lib_path)
        self.foc.init(motor_params, hw)

    def set_pid_gains(self, pid_id, pid_iq, pid_speed, pid_pos):
        self.foc.set_pid_gains(pid_id, pid_iq, pid_speed, pid_pos)

    def run(self, duration, mode, ref_kwargs,
            load_torque=0.0, virtual_elec_offset=0.0):
        """
        Run a closed-loop simulation.

        Parameters
        ----------
        duration            : float — simulation duration (s)
        mode                : int   — FOC_MODE_* constant
        ref_kwargs          : dict  — reference fields (e.g. {'omega_ref': 100.0})
        load_torque         : float — constant mechanical load torque (N·m)
        virtual_elec_offset : float — electrical angle offset injected into the
                              virtual position sensor (rad).  Simulates an
                              uncalibrated encoder: the FOC sees
                              theta_e + virtual_elec_offset while the motor
                              physics use the true theta_e.  Set hw.theta_elec_offset
                              to -virtual_elec_offset to cancel it (calibration goal).

        Returns
        -------
        dict of numpy arrays keyed by signal name.
        """
        dt = self.hw.Ts
        n  = int(duration / dt)
        p  = self.motor_params

        self.foc.reset()
        self.foc.set_mode(mode)
        self.foc.set_ref(**ref_kwargs)

        motor = PMSMModel(self.motor_params)
        keys  = ('time', 'theta_mech', 'omega_mech',
                 'i_u', 'i_v', 'i_w',
                 'i_d', 'i_q', 'v_d', 'v_q',
                 'duty_u', 'duty_v', 'duty_w',
                 'theta_elec_true', 'theta_elec_foc')
        log   = {k: np.zeros(n) for k in keys}

        duty_u = duty_v = duty_w = 0.5   # zero voltage at start

        for i in range(n):
            # Step motor physics with previous cycle's duty cycles
            motor.step(duty_u, duty_v, duty_w, self.hw.v_bus_nominal, dt, load_torque)

            # Virtual sensor: report theta_mech with an electrical angle offset.
            # The mechanical equivalent of the electrical offset is offset/pole_pairs.
            theta_mech_sensor = motor.theta_mech + virtual_elec_offset / p.pole_pairs

            # Run compiled C FOC step with sensed (potentially offset) angle
            duty_u, duty_v, duty_w = self.foc.step(
                motor.i_u, motor.i_v, motor.i_w,
                theta_mech_sensor, motor.omega_mech,
                self.hw.v_bus_nominal)

            ins = self.foc.get_internals()

            log['time'][i]            = i * dt
            log['theta_mech'][i]      = motor.theta_mech
            log['omega_mech'][i]      = motor.omega_mech
            log['i_u'][i]             = motor.i_u
            log['i_v'][i]             = motor.i_v
            log['i_w'][i]             = motor.i_w
            log['i_d'][i]             = ins['i_d']
            log['i_q'][i]             = ins['i_q']
            log['v_d'][i]             = ins['v_d']
            log['v_q'][i]             = ins['v_q']
            log['duty_u'][i]          = duty_u
            log['duty_v'][i]          = duty_v
            log['duty_w'][i]          = duty_w
            log['theta_elec_true'][i] = motor.theta_mech * p.pole_pairs
            log['theta_elec_foc'][i]  = ins['theta_elec']

        return log

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


# ---------------------------------------------------------------------------
# Example usage
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    here     = os.path.dirname(os.path.abspath(__file__))
    lib_name = 'foc_sim.dll' if platform.system() == 'Windows' else 'foc_sim.so'
    lib_path = os.path.join(here, lib_name)

    # Small PMSM — robotic joint servo
    motor_params = MotorParams(
        Rs=0.5, Ld=1e-3, Lq=1e-3, lambda_pm=0.01,
        pole_pairs=7, J=1e-4, Dm=1e-4,
        rated_current=5.0, rated_speed=200.0)

    Ts = 50e-6   # 20 kHz control loop
    hw = HWConfig(Ts=Ts, dead_time=500e-9, v_bus_nominal=24.0,
                  duty_max=0.9, pwm_active_low=False)

    # Current loop gains — bandwidth-based: Kp = ωc·L, Ki = ωc·R·Ts
    wc_i  = 2 * np.pi * 1000          # 1 kHz bandwidth
    Kp_i  = wc_i * motor_params.Ld
    Ki_i  = wc_i * motor_params.Rs * Ts
    Vmax  = hw.v_bus_nominal / 2.0

    pid_id    = PIDGains(Kp=Kp_i, Ki=Ki_i, Kd=0.0, out_min=-Vmax, out_max=Vmax)
    pid_iq    = PIDGains(Kp=Kp_i, Ki=Ki_i, Kd=0.0, out_min=-Vmax, out_max=Vmax)
    pid_speed = PIDGains(Kp=0.3, Ki=0.3 * Ts, Kd=0.0,
                         out_min=-motor_params.rated_current,
                         out_max= motor_params.rated_current)
    pid_pos   = PIDGains(Kp=30.0, Ki=0.0, Kd=0.0,
                         out_min=-motor_params.rated_speed,
                         out_max= motor_params.rated_speed)

    sim = FOCSimulator(motor_params, hw, lib_path)
    sim.set_pid_gains(pid_id, pid_iq, pid_speed, pid_pos)

    # --- Velocity step, no angle offset ---
    log = sim.run(duration=0.1, mode=FOC_MODE_VELOCITY,
                  ref_kwargs={'omega_ref': 100.0, 'i_d_ref': 0.0})
    FOCSimulator.plot(log, title='Velocity step — 0 → 100 rad/s (no offset)')

    # --- Same run with a 30° electrical angle offset (calibration test) ---
    # log_offset = sim.run(duration=0.1, mode=FOC_MODE_VELOCITY,
    #                      ref_kwargs={'omega_ref': 100.0, 'i_d_ref': 0.0},
    #                      virtual_elec_offset=np.radians(30.0))
    # FOCSimulator.plot(log_offset, title='Velocity step — 30° angle offset')
