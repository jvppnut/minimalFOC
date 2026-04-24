"""
Voltage control with uncalibrated electrical angle — v_d = 0, v_q = ±2 V, 5 Hz.

Same as test_voltage_square but a 30° electrical angle offset is injected into
the virtual position sensor. The FOC sees a rotor position 30° ahead of the
true position, so the voltage vector is misaligned: torque production degrades
and a d-axis current component appears.

The bottom plot shows the true vs. FOC-perceived electrical angle to make the
offset visible.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import platform
import numpy as np
import matplotlib.pyplot as plt
from simulator import (MotorParams, HWConfig, PIDGains,
                       FOCSimulator, FOC_MODE_VOLTAGE)

# ---------------------------------------------------------------------------
# Library path
# ---------------------------------------------------------------------------
_here    = os.path.dirname(os.path.abspath(__file__))
_lib     = 'foc_sim.dll' if platform.system() == 'Windows' else 'foc_sim.so'
LIB_PATH = os.path.join(_here, '..', _lib)

# ---------------------------------------------------------------------------
# Motor and hardware parameters
# ---------------------------------------------------------------------------
motor = MotorParams(
    Rs=0.5, Ld=1e-3, Lq=1e-3, lambda_pm=0.01,
    pole_pairs=7, J=1e-4, Dm=1e-4,
    rated_current=5.0, rated_speed=200.0)

Ts_hw  = 50e-6    # 20 kHz control loop
Ts_sim =  5e-6    # 200 kHz physics integration

hw = HWConfig(Ts=Ts_hw, dead_time=500e-9, v_bus_nominal=24.0,
              duty_max=0.9, pwm_active_low=False)

# ---------------------------------------------------------------------------
# PID gains
# ---------------------------------------------------------------------------
wc_i = 2 * np.pi * 1000
Kp_i = wc_i * motor.Ld
Ki_i = wc_i * motor.Rs * Ts_hw
Vmax = hw.v_bus_nominal / 2.0

pid_id    = PIDGains(Kp=Kp_i, Ki=Ki_i, Kd=0.0, out_min=-Vmax, out_max=Vmax)
pid_iq    = PIDGains(Kp=Kp_i, Ki=Ki_i, Kd=0.0, out_min=-Vmax, out_max=Vmax)
pid_speed = PIDGains(Kp=0.0,  Ki=0.0,  Kd=0.0, out_min=-motor.rated_current, out_max=motor.rated_current)
pid_pos   = PIDGains(Kp=0.0,  Ki=0.0,  Kd=0.0, out_min=-motor.rated_speed,   out_max=motor.rated_speed)

# ---------------------------------------------------------------------------
# Reference waveform and angle offset
# ---------------------------------------------------------------------------
FREQ          = 5.0              # Hz
VQ_AMP        = 2.0              # V
ANGLE_OFFSET  = np.radians(30.0) # 30° electrical misalignment

def ref_fn(t):
    vq = VQ_AMP if (t * FREQ % 1.0) < 0.5 else -VQ_AMP
    return {'v_d_ref': 0.0, 'v_q_ref': vq}

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
sim = FOCSimulator(motor, hw, LIB_PATH, Ts_sim=Ts_sim)
sim.set_pid_gains(pid_id, pid_iq, pid_speed, pid_pos)

log = sim.run(duration=0.5, mode=FOC_MODE_VOLTAGE, ref_fn=ref_fn,
              virtual_elec_offset=ANGLE_OFFSET)

# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
t_ms = log['time'] * 1e3
v_u  = (log['duty_u'] - 0.5) * hw.v_bus_nominal
v_v  = (log['duty_v'] - 0.5) * hw.v_bus_nominal
v_w  = (log['duty_w'] - 0.5) * hw.v_bus_nominal

fig, axes = plt.subplots(6, 1, figsize=(11, 15), sharex=True)
fig.suptitle('Voltage control — 30° angle offset  (v_q = ±2 V, 5 Hz)')

axes[0].plot(t_ms, log['v_q_ref'], label='v_q_ref', linestyle='--', color='gray')
axes[0].plot(t_ms, log['v_q'],     label='v_q (FOC output)')
axes[0].set_ylabel('q-axis voltage (V)')
axes[0].legend()
axes[0].grid(True)

axes[1].plot(t_ms, log['i_d'], label='i_d')
axes[1].plot(t_ms, log['i_q'], label='i_q')
axes[1].set_ylabel('dq currents (A)')
axes[1].legend()
axes[1].grid(True)

axes[2].plot(t_ms, log['i_u'], label='i_u')
axes[2].plot(t_ms, log['i_v'], label='i_v')
axes[2].plot(t_ms, log['i_w'], label='i_w')
axes[2].set_ylabel('Phase currents (A)')
axes[2].legend()
axes[2].grid(True)

axes[3].plot(t_ms, v_u, label='v_u')
axes[3].plot(t_ms, v_v, label='v_v')
axes[3].plot(t_ms, v_w, label='v_w')
axes[3].set_ylabel('Phase voltages (V)')
axes[3].legend()
axes[3].grid(True)

axes[4].plot(t_ms, log['omega_mech'])
axes[4].set_ylabel('Speed (rad/s)')
axes[4].grid(True)

axes[5].plot(t_ms, np.degrees(log['theta_elec_true']), label='θ_e true')
axes[5].plot(t_ms, np.degrees(log['theta_elec_foc']),  label='θ_e FOC', linestyle='--')
axes[5].set_ylabel('Electrical angle (°)')
axes[5].set_xlabel('Time (ms)')
axes[5].legend()
axes[5].grid(True)

plt.tight_layout()
plt.show()
