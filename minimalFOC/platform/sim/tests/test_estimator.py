"""
Estimator test — 14-bit encoder quantization, velocity step.

Verifies:
  1. omega_mech_est converges to omega_mech truth at steady state.
  2. The IIR LPF suppresses quantization noise in the backward-difference
     omega estimate; the raw (unfiltered) staircase derivative is plotted
     alongside the LPF output to make this visible.
  3. theta_mech_est correctly tracks multi-turn position.

Scenario
--------
  - Step velocity reference to OMEGA_REF rad/s at t=0.
  - Hold for DURATION seconds.
  - Sensor: 14-bit absolute encoder (16384 counts/rev, ~0.022°/LSB).

Plots
-----
  1. Speed — omega_mech (truth) vs omega_mech_est (full run).
  2. Speed (full run) — raw backward-difference omega, LPF-filtered estimate,
     and physics truth side-by-side; transient and noise both visible.
  3. Multi-turn position — theta_mech (truth) vs theta_mech_est.
  4. Single-turn angle detail (last ZOOM_MS ms) — quantized staircase
     vs physics truth; LSB boundaries shown as horizontal dashes.
  5. Estimation errors — omega and theta residuals over the full run.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import platform
import numpy as np
import matplotlib.pyplot as plt
from simulator import (MotorParams, HWConfig, PIDGains, FOCSimulator,
                       VirtualPositionSensor, FOC_MODE_VELOCITY)

# ---------------------------------------------------------------------------
# Library path
# ---------------------------------------------------------------------------
_here    = os.path.dirname(os.path.abspath(__file__))
_lib     = 'foc_sim.dll' if platform.system() == 'Windows' else 'foc_sim.so'
LIB_PATH = os.path.join(_here, '..', _lib)

# ---------------------------------------------------------------------------
# Motor and hardware parameters
# ---------------------------------------------------------------------------
motor = MotorParams(                    # CubeMars AK60-6 V1.1 (KV80)
    Rs=0.101, Ld=69e-6, Lq=69e-6,      # Y-equiv from delta LL measurements
    lambda_pm=0.00371,                  # derived from Kt=0.078 Nm/A
    pole_pairs=14,
    J=2.435e-5,                         # rotor inertia from spec (kg·m²)
    Dm=2.5e-5,                          # estimated (0.25x placeholder)
    rated_current=6.5, rated_speed=264.0)

Ts_hw  = 50e-6   # 20 kHz control loop
Ts_sim =  5e-6   # 200 kHz physics integration

hw = HWConfig(Ts=Ts_hw, dead_time=500e-9, v_bus_nominal=24.0,
              duty_max=0.9, pwm_active_low=False)

# ---------------------------------------------------------------------------
# PID gains
# ---------------------------------------------------------------------------
wc_i = 2 * np.pi * 1000   # 1 kHz current loop bandwidth
Kp_i = wc_i * motor.Ld
Ki_i = wc_i * motor.Rs * Ts_hw
Vmax = hw.v_bus_nominal / 2.0

pid_id = PIDGains(Kp=Kp_i, Ki=Ki_i, Kd=0.0, out_min=-Vmax, out_max=Vmax)
pid_iq = PIDGains(Kp=Kp_i, Ki=Ki_i, Kd=0.0, out_min=-Vmax, out_max=Vmax)

# Speed loop — output is i_q reference.
# Gain chosen for ~10 rad/s closed-loop bandwidth.
Kp_spd = 0.033  # A / (rad/s)  — rescaled for AK60-6 J and lambda_pm
Ki_spd = 0.0013 # A / (rad/s) per step
pid_speed = PIDGains(Kp=Kp_spd, Ki=Ki_spd, Kd=0.0,
                     out_min=-motor.rated_current, out_max=motor.rated_current)
pid_pos   = PIDGains(Kp=0.0, Ki=0.0, Kd=0.0,
                     out_min=-motor.rated_speed, out_max=motor.rated_speed)

# ---------------------------------------------------------------------------
# Estimator LPF — alpha = exp(-2π·fc·Ts), fc ≈ 200 Hz @ Ts = 50 µs
# ---------------------------------------------------------------------------
ESTIMATOR_ALPHA = 0.9394

# ---------------------------------------------------------------------------
# Scenario
# ---------------------------------------------------------------------------
ENCODER_BITS = 14         # 16384 counts / rev
OMEGA_REF    = 50.0       # rad/s  (~478 RPM)
STEP_TIME    = 2.0        # s — velocity step applied at this time
DURATION     = 3.5        # s — total run (STEP_TIME + 1.5 s to capture full transient)
ZOOM_MS      = 20.0       # ms — detail window taken from end of run

def ref_fn(t):
    omega = OMEGA_REF if t >= STEP_TIME else 0.0
    return {'omega_ref': omega}

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
sensor = VirtualPositionSensor(motor.pole_pairs, bits=ENCODER_BITS)
sim    = FOCSimulator(motor, hw, LIB_PATH, sensor=sensor, Ts_sim=Ts_sim,
                      estimator_lpf_alpha=ESTIMATOR_ALPHA)
sim.set_pid_gains(pid_id, pid_iq, pid_speed, pid_pos)

log = sim.run(duration=DURATION, mode=FOC_MODE_VELOCITY, ref_fn=ref_fn)

# ---------------------------------------------------------------------------
# Derived signals
# ---------------------------------------------------------------------------
t_ms    = log['time'] * 1e3
n_zoom  = int(ZOOM_MS * 1e-3 / Ts_hw)          # steps in zoom window
n_zoom4 = max(1, n_zoom // 4)                   # shorter window for staircase

# Raw omega — backward difference of unwrapped quantized angle, before LPF.
# This is what the estimator sees before filtering; it shows quantization noise.
theta_raw_unwrap = np.unwrap(log['theta_mech_raw'])
omega_raw        = np.concatenate([[0.0],
                                   np.diff(theta_raw_unwrap) / Ts_hw])

# Steady-state metrics (last 10 % of run)
ss_start = int(0.9 * len(t_ms))
omega_err_ss  = log['omega_mech_est'][ss_start:] - log['omega_mech'][ss_start:]
theta_err_full = log['theta_mech_est'] - log['theta_mech']

print(f"Encoder       : {ENCODER_BITS}-bit  ({1 << ENCODER_BITS} counts/rev,"
      f"  LSB = {np.degrees(2*np.pi / (1 << ENCODER_BITS)):.4f}°)")
print(f"LPF alpha     : {ESTIMATOR_ALPHA}  (fc ~ {-np.log(ESTIMATOR_ALPHA)/(2*np.pi*Ts_hw):.0f} Hz)")
print(f"Speed ref     : {OMEGA_REF} rad/s")
print(f"Steady-state omega error : mean = {omega_err_ss.mean():+.4f} rad/s,"
      f"  std = {omega_err_ss.std():.4f} rad/s")
print(f"Final theta error        : {theta_err_full[-1]:+.5f} rad"
      f"  ({np.degrees(theta_err_full[-1]):+.4f}°)")

# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
LSB_RAD = 2.0 * np.pi / (1 << ENCODER_BITS)

fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=False)
fig.suptitle(
    f'Estimator — {ENCODER_BITS}-bit encoder,  ω_ref = {OMEGA_REF} rad/s,  '
    f'LPF α = {ESTIMATOR_ALPHA} (fc ≈ {-np.log(ESTIMATOR_ALPHA)/(2*np.pi*Ts_hw):.0f} Hz)',
    fontsize=11)

step_ms = STEP_TIME * 1e3   # step time in ms for vertical markers

def mark_step(ax):
    ax.axvline(step_ms, color='gray', linewidth=0.8, linestyle='--', alpha=0.7,
               label=f'step @ {STEP_TIME:.0f} s')

# -- 1. Full-run speed -------------------------------------------------------
ax = axes[0]
ax.plot(t_ms, log['omega_mech'],     label='ω truth (physics)',  alpha=0.7)
ax.plot(t_ms, log['omega_mech_est'], label='ω est (LPF)',
        linestyle='--', color='tab:orange')
ax.axhline(OMEGA_REF, color='gray', linewidth=0.8, linestyle=':', label='ω_ref')
mark_step(ax)
ax.set_ylabel('Speed (rad/s)')
ax.set_xlabel('Time (ms)')
ax.legend(fontsize=8)
ax.grid(True)
ax.set_title('Speed — full run', fontsize=9)

# -- 2. Speed — raw vs LPF vs truth (full run) ------------------------------
ax = axes[1]
mark_step(ax)
ax.plot(t_ms, omega_raw,             label='ω raw (backward diff, quantized)',
        color='tab:red',   alpha=0.5, linewidth=0.8)
ax.plot(t_ms, log['omega_mech'],     label='ω truth',
        color='tab:blue',  alpha=0.8)
ax.plot(t_ms, log['omega_mech_est'], label='ω est (LPF)',
        color='tab:orange', linewidth=1.5, linestyle='--')
ax.set_ylabel('Speed (rad/s)')
ax.set_xlabel('Time (ms)')
ax.legend(fontsize=8)
ax.grid(True)
ax.set_title('Speed — raw vs LPF vs truth  (full run, quantization noise visible)', fontsize=9)

# -- 3. Multi-turn position (full run) --------------------------------------
ax = axes[2]
ax.plot(t_ms, log['theta_mech'],     label='θ truth (multi-turn)',  alpha=0.7)
ax.plot(t_ms, log['theta_mech_est'], label='θ est',
        linestyle='--', color='tab:orange')
mark_step(ax)
ax.set_ylabel('θ_mech (rad)')
ax.set_xlabel('Time (ms)')
ax.legend(fontsize=8)
ax.grid(True)
ax.set_title('Multi-turn position', fontsize=9)

# -- 4. Single-turn staircase detail (shorter zoom) --------------------------
ax  = axes[3]
sl4 = slice(-n_zoom4, None)
t_z4 = t_ms[sl4]
ax.plot(t_z4, np.degrees(log['theta_mech_single'][sl4]),
        label='θ truth (single-turn)', color='tab:blue', alpha=0.8)
ax.step(t_z4, np.degrees(log['theta_mech_raw'][sl4]),
        label='θ quantized (encoder)', color='tab:red',
        where='post', linewidth=1.0, alpha=0.9)
# Mark LSB boundaries in the visible range
y_lo = np.degrees(log['theta_mech_raw'][sl4].min())
y_hi = np.degrees(log['theta_mech_raw'][sl4].max()) + np.degrees(LSB_RAD)
lsb_deg = np.degrees(LSB_RAD)
for boundary in np.arange(np.floor(y_lo / lsb_deg) * lsb_deg,
                           y_hi + lsb_deg, lsb_deg):
    ax.axhline(boundary, color='gray', linewidth=0.4, linestyle='--', alpha=0.5)
ax.set_ylabel('θ_mech (deg)')
ax.set_xlabel('Time (ms)')
ax.legend(fontsize=8)
ax.grid(True)
ax.set_title(f'Encoder staircase detail — last {ZOOM_MS/4:.1f} ms'
             f'  (LSB = {lsb_deg:.3f}°)', fontsize=9)

# -- 5. Estimation errors (full run) -----------------------------------------
ax = axes[4]
mark_step(ax)
omega_err = log['omega_mech_est'] - log['omega_mech']
ax.plot(t_ms, omega_err, label='ω error (est − truth)', color='tab:purple', linewidth=0.8)
ax.axhline(0, color='k', linewidth=0.5)
ax.set_ylabel('ω error (rad/s)')
ax.set_xlabel('Time (ms)')
ax.legend(fontsize=8, loc='upper right')
ax.grid(True)

ax2 = ax.twinx()
ax2.plot(t_ms, np.degrees(theta_err_full),
         label='θ error (est − truth)', color='tab:green',
         linewidth=0.8, alpha=0.7, linestyle='--')
ax2.set_ylabel('θ error (deg)', color='tab:green')
ax2.tick_params(axis='y', labelcolor='tab:green')
ax2.legend(fontsize=8, loc='lower right')

axes[4].set_title('Estimation errors', fontsize=9)

plt.tight_layout()
plt.show()
