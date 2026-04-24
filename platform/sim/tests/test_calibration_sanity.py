"""
Headless sanity check for the calibration test.

Verifies:
  - Phase 1 ends at the expected time.
  - Calibration loop terminates after exactly CAL_SETTLE / Ts_hw steps.
  - FOC_Sim_GetMode() correctly detects the C-side mode switch back to VOLTAGE.
  - Post-calibration i_d is near zero (angle frame corrected).
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import platform
import numpy as np
from simulator import (MotorParams, HWConfig, PIDGains, FOCSimulator,
                       VirtualPositionSensor,
                       FOC_MODE_VOLTAGE, FOC_MODE_CALIBRATE)

# ---------------------------------------------------------------------------
# Parameters (must match test_calibration.py)
# ---------------------------------------------------------------------------
motor = MotorParams(
    Rs=0.5, Ld=1e-3, Lq=1e-3, lambda_pm=0.01,
    pole_pairs=7, J=1e-4, Dm=1e-4,
    rated_current=5.0, rated_speed=200.0)

Ts_hw  = 50e-6
Ts_sim =  5e-6

hw = HWConfig(Ts=Ts_hw, dead_time=500e-9, v_bus_nominal=24.0,
              duty_max=0.9, pwm_active_low=False)

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

_here    = os.path.dirname(os.path.abspath(__file__))
_lib     = 'foc_sim.dll' if platform.system() == 'Windows' else 'foc_sim.so'
LIB_PATH = os.path.join(_here, '..', _lib)

ELEC_OFFSET_RAD = np.radians(30.0)
CAL_VD          = 2.0
CAL_SETTLE      = 0.5
DUR_PRE         = 0.5
DUR_POST        = 0.5

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
sensor = VirtualPositionSensor(motor.pole_pairs, elec_offset_rad=ELEC_OFFSET_RAD)
sim = FOCSimulator(motor, hw, LIB_PATH, sensor=sensor, Ts_sim=Ts_sim)
sim.set_pid_gains(pid_id, pid_iq, pid_speed, pid_pos)
sim.reset()

# Phase 1 — misaligned
sim.foc.set_mode(FOC_MODE_VOLTAGE)
for _ in range(int(DUR_PRE / Ts_hw)):
    sim.foc.set_ref(v_d_ref=0.0, v_q_ref=2.0)
    sim.step()
print(f"Phase 1 done : t = {sim._t * 1e3:.1f} ms  (expected {DUR_PRE*1e3:.1f} ms)")

# Phase 2 — calibration
sim.foc.calibrate(v_cal=CAL_VD, settle_time_s=CAL_SETTLE)
expected_cal_steps = int(CAL_SETTLE / Ts_hw)
cal_steps = 0
while sim.foc.get_mode() == FOC_MODE_CALIBRATE:
    sim.step()
    cal_steps += 1
print(f"Calibration  : {cal_steps} steps / {cal_steps * Ts_hw * 1e3:.1f} ms"
      f"  (expected {expected_cal_steps} / {CAL_SETTLE*1e3:.1f} ms)"
      f"  t = {sim._t * 1e3:.1f} ms")

# Phase 3 — calibrated
sim.foc.reset()
sim.foc.set_mode(FOC_MODE_VOLTAGE)
last = None
for _ in range(int(DUR_POST / Ts_hw)):
    sim.foc.set_ref(v_d_ref=0.0, v_q_ref=2.0)
    last = sim.step()
print(f"Phase 3 done : t = {sim._t * 1e3:.1f} ms  (expected {(DUR_PRE+CAL_SETTLE+DUR_POST)*1e3:.1f} ms)")
print(f"Final state  : i_d = {last['i_d']:+.4f} A,  i_q = {last['i_q']:+.4f} A")
print(f"               theta_elec_true = {np.degrees(last['theta_elec_true']):+.2f}°,"
      f"  theta_elec_foc = {np.degrees(last['theta_elec_foc']):+.2f}°,"
      f"  error = {np.degrees(last['theta_elec_foc'] - last['theta_elec_true']):+.2f}°")
