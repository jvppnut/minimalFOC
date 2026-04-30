"""
Electrical angle offset calibration test.

Three phases on one continuous time axis:
  1. Pre-calibration  — v_d=0, v_q=±2V square 5 Hz with 30° electrical offset
  2. Calibration      — FOC_Calibrate() forces d-axis along phase U; rotor settles
  3. Post-calibration — same square wave; FOC now uses corrected angle

Expected observations
---------------------
- Phase 1: i_d/i_q are distorted because the angle frame is wrong.
- Phase 2: motor is driven to theta_e = 0 by the calibration voltage.
- Phase 3: i_d stays near zero, i_q tracks the q-axis voltage cleanly.
- theta_elec_foc error (FOC θ − true θ) should collapse to ~0 after cal.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import platform
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from simulator import (MotorParams, HWConfig, PIDGains, FOCSimulator,
                       VirtualPositionSensor,
                       FOC_MODE_VOLTAGE, FOC_MODE_CALIBRATE, _stack_samples)

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
# PID gains (current loop only)
# ---------------------------------------------------------------------------
wc_i = 2 * np.pi * 1000
Kp_i = wc_i * motor.Ld
Ki_i = wc_i * motor.Rs * Ts_hw
Vmax = hw.v_bus_nominal / 2.0

pid_id    = PIDGains(Kp=Kp_i, Ki=Ki_i, Kd=0.0, out_min=-Vmax, out_max=Vmax)
pid_iq    = PIDGains(Kp=Kp_i, Ki=Ki_i, Kd=0.0, out_min=-Vmax, out_max=Vmax)
pid_speed = PIDGains(Kp=0.0,  Ki=0.0,  Kd=0.0, out_min=-motor.rated_current,
                     out_max=motor.rated_current)
pid_pos   = PIDGains(Kp=0.0,  Ki=0.0,  Kd=0.0, out_min=-motor.rated_speed,
                     out_max=motor.rated_speed)

# ---------------------------------------------------------------------------
# Scenario parameters
# ---------------------------------------------------------------------------
ELEC_OFFSET_DEG = 30.0                          # simulated encoder misalignment
ELEC_OFFSET_RAD = np.radians(ELEC_OFFSET_DEG)  # electrical radians

FREQ       = 5.0   # Hz
VQ_AMP     = 2.0   # V
CAL_VD     = 2.0   # V  — d-axis alignment voltage
CAL_SETTLE = 0.5   # s  — calibration settle time

DUR_PRE  = 0.5     # s — misaligned square wave
DUR_POST = 0.5     # s — calibrated square wave

def ref_fn(t):
    vq = VQ_AMP if (t * FREQ % 1.0) < 0.5 else -VQ_AMP
    return {'v_d_ref': 0.0, 'v_q_ref': float(vq)}

# ---------------------------------------------------------------------------
# Build simulator and initial setup
# ---------------------------------------------------------------------------
sensor = VirtualPositionSensor(motor.pole_pairs, elec_offset_rad=ELEC_OFFSET_RAD)
sim = FOCSimulator(motor, hw, LIB_PATH, sensor=sensor, Ts_sim=Ts_sim)
sim.set_pid_gains(pid_id, pid_iq, pid_speed, pid_pos)
sim.reset()

# ---- Phase 1: misaligned voltage square -----------------------------------
sim.foc.set_mode(FOC_MODE_VOLTAGE)

samples_pre = []
n_pre = int(DUR_PRE / Ts_hw)
for _ in range(n_pre):
    ref = ref_fn(sim._t)
    sim.foc.set_ref(**ref)
    s = sim.step()
    s['v_q_ref'] = ref['v_q_ref']
    samples_pre.append(s)

t_cal_start = sim._t

# ---- Phase 2: calibration -------------------------------------------------
sim.foc.calibrate(v_cal=CAL_VD, settle_time_s=CAL_SETTLE)

samples_cal = []
while sim.foc.get_mode() == FOC_MODE_CALIBRATE:
    s = sim.step()
    s['v_q_ref'] = 0.0
    samples_cal.append(s)

t_cal_end = sim._t

# ---- Phase 3: calibrated voltage square -----------------------------------
sim.foc.reset()   # clear PID integrators before resuming normal control
sim.foc.set_mode(FOC_MODE_VOLTAGE)

samples_post = []
n_post = int(DUR_POST / Ts_hw)
for _ in range(n_post):
    ref = ref_fn(sim._t)
    sim.foc.set_ref(**ref)
    s = sim.step()
    s['v_q_ref'] = ref['v_q_ref']
    samples_post.append(s)

# ---------------------------------------------------------------------------
# Merge all phases into one continuous log
# ---------------------------------------------------------------------------
log = _stack_samples(samples_pre + samples_cal + samples_post)

t_ms         = log['time'] * 1e3
t_cal_start_ms = t_cal_start * 1e3
t_cal_end_ms   = t_cal_end   * 1e3

#theta_err_deg = np.degrees((log['theta_elec_foc'] - log['theta_elec_true']) % (2.0 * np.pi))  # wrapped [0, 2pi): square waves post-cal
#theta_err_deg = np.degrees(log['theta_elec_foc'] - log['theta_elec_true'])                     # raw subtraction: square waves pre/during-cal
diff = log['theta_elec_foc'] - log['theta_elec_true']
theta_err_deg = np.degrees(np.arctan2(np.sin(diff), np.cos(diff)))
# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(6, 1, figsize=(13, 16), sharex=True)
fig.suptitle(
    f'Electrical angle calibration — {ELEC_OFFSET_DEG}° offset, '
    f'v_q = ±{VQ_AMP}V square {FREQ}Hz',
    fontsize=12)

def shade_phases(ax):
    ax.axvspan(0,              t_cal_start_ms, alpha=0.07, color='red',   label='pre-cal')
    ax.axvspan(t_cal_start_ms, t_cal_end_ms,   alpha=0.10, color='blue',  label='calibration')
    ax.axvspan(t_cal_end_ms,   t_ms[-1],        alpha=0.07, color='green', label='post-cal')
    ax.axvline(t_cal_start_ms, color='blue',  linestyle='--', linewidth=0.8)
    ax.axvline(t_cal_end_ms,   color='green', linestyle='--', linewidth=0.8)

# -- v_q reference vs actual ------------------------------------------------
axes[0].plot(t_ms, log['v_q_ref'], linestyle='--', color='gray', label='v_q_ref')
axes[0].plot(t_ms, log['v_q'],                                   label='v_q (FOC out)')
shade_phases(axes[0])
axes[0].set_ylabel('q-axis voltage (V)')
axes[0].legend(loc='upper right', fontsize=8)
axes[0].grid(True)

# -- dq currents ------------------------------------------------------------
axes[1].plot(t_ms, log['i_d'], label='i_d')
axes[1].plot(t_ms, log['i_q'], label='i_q')
shade_phases(axes[1])
axes[1].set_ylabel('dq currents (A)')
axes[1].legend(loc='upper right', fontsize=8)
axes[1].grid(True)

# -- Phase currents ---------------------------------------------------------
axes[2].plot(t_ms, log['i_u'], label='i_u')
axes[2].plot(t_ms, log['i_v'], label='i_v')
axes[2].plot(t_ms, log['i_w'], label='i_w')
shade_phases(axes[2])
axes[2].set_ylabel('Phase currents (A)')
axes[2].legend(loc='upper right', fontsize=8)
axes[2].grid(True)

# -- Single-turn mechanical angle [0°, 360°) --------------------------------
# Shows the raw encoder wrap behaviour and the constant offset the sensor injects.
axes[3].plot(t_ms, np.degrees(log['theta_mech_single']),
             label='true θ_mech (physics)', alpha=0.7)
axes[3].plot(t_ms, np.degrees(log['theta_mech_raw']),
             label='sensor θ_mech (with offset)', linestyle='--', alpha=0.9)
shade_phases(axes[3])
axes[3].set_ylabel('Mechanical angle (°)\n[0°, 360°)')
axes[3].legend(loc='upper right', fontsize=8)
axes[3].grid(True)

# -- Electrical angles (true vs FOC) ------------------------------------------
axes[4].plot(t_ms, np.degrees(log['theta_elec_true']), label='θ_e true', alpha=0.8)
axes[4].plot(t_ms, np.degrees(log['theta_elec_foc']),  label='θ_e FOC',
             linestyle='--', alpha=0.9)
shade_phases(axes[4])
axes[4].set_ylabel('Electrical angle (°)\n[0°, 360°)')
axes[4].legend(loc='upper right', fontsize=8)
axes[4].grid(True)

# -- Electrical angle error — collapses to ~0° after calibration --------------
axes[5].plot(t_ms, theta_err_deg, color='darkorange')
axes[5].axhline(ELEC_OFFSET_DEG, color='gray', linewidth=0.8, linestyle=':',
                label=f'pre-cal offset ({ELEC_OFFSET_DEG}°)')
axes[5].axhline(0, color='k', linewidth=0.6, linestyle=':')
shade_phases(axes[5])
axes[5].set_ylabel('θ_e error (°)\n(FOC − true)')
axes[5].set_xlabel('Time (ms)')
axes[5].legend(loc='upper right', fontsize=8)
axes[5].grid(True)

# Phase legend (shared)
patches = [
    mpatches.Patch(color='red',   alpha=0.4, label='Pre-cal (misaligned)'),
    mpatches.Patch(color='blue',  alpha=0.5, label='Calibration'),
    mpatches.Patch(color='green', alpha=0.4, label='Post-cal (corrected)'),
]
fig.legend(handles=patches, loc='lower center', ncol=3, fontsize=9,
           bbox_to_anchor=(0.5, 0.0))
fig.subplots_adjust(bottom=0.06)

plt.tight_layout(rect=[0, 0.04, 1, 1])
plt.show()
