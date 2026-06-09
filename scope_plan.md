# FOC Scope Tool — Design Plan

Real-time oscilloscope and control panel for the minimalFOC system, running on the host PC over UART.

---

## Communication Architecture

### MCU → PC (telemetry)

Extend the existing binary frame to include references and active mode.

**Current frame (47 bytes):** 11 × float32 + header + CRC  
**New frame (64 bytes):** 15 × float32 + 1 × uint8 mode + header + CRC

```
[0xAA][0x55][15 × float32 LE = 60 B][uint8 mode][XOR CRC]  — 64 bytes total
```

Fields (in order):
1. `theta_mech`
2. `omega_mech`
3. `theta_elec`
4. `i_u`, `i_v`, `i_w`
5. `i_d`, `i_q`
6. `v_d`, `v_q`  ← post-limiter commanded values (motor.out); v_d_ref/v_q_ref omitted as redundant
7. `i_d_ref`, `i_q_ref`
8. `omega_ref`
9. `theta_ref`
10. `isr_us`

Rate: 1 kHz (decimated ×20 from 20 kHz ISR). Baud: 921600.

### PC → MCU (commands)

Lightweight fixed-length binary frames over the same UART.

```
[0xBB][CMD][float32 payload LE = 4 B][XOR CRC]  — 7 bytes
```

| CMD byte | Command        | Payload             |
|----------|----------------|---------------------|
| 0x01     | SET_MODE       | mode as float (0–3) |
| 0x02     | SET_VD_REF     | v_d_ref (V)         |
| 0x03     | SET_VQ_REF     | v_q_ref (V)         |
| 0x04     | SET_ID_REF     | i_d_ref (A)         |
| 0x05     | SET_IQ_REF     | i_q_ref (A)         |
| 0x06     | SET_OMEGA_REF  | omega_ref (rad/s)   |
| 0x07     | SET_THETA_REF  | theta_ref (rad)     |
| 0x08     | STOP           | 0x00000000          |
| 0x09     | RESUME         | 0x00000000          |
| 0x0A     | FGEN_WAVE      | waveform (0=step, 1=square, 2=triangle, 3=sine) as float |
| 0x0B     | FGEN_FREQ      | frequency (Hz)      |
| 0x0C     | FGEN_AMP       | amplitude (peak, same units as active ref) |
| 0x0D     | FGEN_OFFSET    | DC offset (same units as active ref) |
| 0x0E     | FGEN_ENABLE    | 1.0=enable, 0.0=disable |

---

## Host Application Stack

| Layer       | Choice                              |
|-------------|-------------------------------------|
| GUI         | PyQt5                               |
| Plots       | pyqtgraph (fast real-time, not matplotlib) |
| Serial      | pyserial                            |
| Language    | Python 3.x                          |

---

## Display Layout

```
┌──────────────────────────────────────────────────────────────────────┐
│  PORT: [COM3 ▼]  [Connect]   MODE: [Voltage ▼]   Window: [1.0 s ▼]  │
│  Vd: [0.0]  Vq: [1.0]   [Apply]              [■ STOP]    [▶ RUN]    │
├──────────────────────────────────────────────────────────────────────┤
│  FUNCTION GENERATOR                                       [● Enable] │
│  Wave: [Sine ▼]  Freq: [1.0] Hz  Amp: [0.5]  Offset: [0.0]          │
└──────────────────────────────────────────────────────────────────────┘
┌──────────────────────────┐  ┌───────────────────────────────────────┐
│  UVW Currents (A)        │  │  dq Currents + Refs (A)               │
│  i_u, i_v, i_w           │  │  i_d, i_q, i_d_ref, i_q_ref          │
└──────────────────────────┘  └───────────────────────────────────────┘
┌──────────────────────────┐  ┌───────────────────────────────────────┐
│  dq Voltages (V)         │  │  Angles (rad)                         │
│  v_d, v_q (post-limiter) │  │  theta_mech (unwrapped), theta_elec   │
└──────────────────────────┘  └───────────────────────────────────────┘
┌──────────────────────────┐  ┌───────────────────────────────────────┐
│  omega_mech (rad/s)      │  │  ISR time (µs)                        │
└──────────────────────────┘  └───────────────────────────────────────┘
┌──────────────────────────────────────────────────────────────────────┐
│  LIVE VALUES                                                         │
│  θ_mech  ω_mech  θ_elec  i_u   i_v   i_w   i_d   i_q               │
│  v_d     v_q     i_d_ref i_q_ref  ω_ref  θ_ref  isr_us  mode        │
└──────────────────────────────────────────────────────────────────────┘
```

Each plot:
- Scrolling time window — adjustable via **Window** dropdown: 0.2 s / 0.5 s / 1 s / 2 s / 5 s / 10 s / 20 s
- Toggleable traces (click legend to show/hide)
- Y-axis autoscale

---

## Control Panel Behaviour

- **Mode selector**: Voltage / Torque / Velocity / Position dropdown.
  On change:
  1. Sends `STOP` immediately.
  2. Zeros all reference fields in the UI.
  3. Sends all-zero refs (`SET_VD_REF 0`, `SET_VQ_REF 0`, etc.) to the MCU.
  4. Switches to the new mode (`SET_MODE`).
  5. Shows only the relevant reference fields for the new mode.
  6. Leaves motor **stopped** — user must hit **RUN** explicitly to resume.
  This prevents the motor from jumping to a stale reference on mode change.
  The `SET_MODE` command on the MCU side must also call `FOC_Reset()` to clear
  PID integrators — otherwise a previously wound-up integrator from an abandoned
  mode could cause a torque spike when that mode is re-entered.
- **Reference fields**: float entries. Only active-mode refs shown (e.g. Voltage mode shows Vd/Vq; Torque shows Id_ref/Iq_ref; etc.)
- **Apply**: sends current ref values as individual SET_* commands. Only active when motor is running.
- **STOP**: sends `STOP` command. MCU zeros outputs and disables INLx.
- **RUN**: sends `RESUME`. MCU re-enables switching. Only available after a mode is selected and refs are set.
- **Function generator panel**:
  - **Wave selector**: Step / Square / Triangle / Sine dropdown → sends `FGEN_WAVE`.
  - **Freq / Amp / Offset fields**: sent on change → `FGEN_FREQ`, `FGEN_AMP`, `FGEN_OFFSET`.
  - **Enable toggle**: sends `FGEN_ENABLE 1` or `0`. When disabled the MCU uses the static ref from the Apply fields instead.
  - The function generator drives whichever reference is active for the current mode (e.g. `i_q_ref` in Torque mode, `omega_ref` in Velocity mode). The offset acts as the DC bias around which the waveform oscillates.
  - On mode switch, function generator is automatically disabled and must be re-enabled manually — consistent with the all-refs-to-zero safety rule.

---

## MCU Changes Needed

1. **Extend `log_pack()`** — add `i_d_ref`, `i_q_ref`, `omega_ref`, `theta_ref`, mode byte; drop `v_d_ref`/`v_q_ref`; update `LOG_FRAME_BYTES` to 64.
2. **UART RX ring buffer** — small circular buffer fed by USART1 RX DMA (already initialized).
3. **Command parser** — called from main loop; detects 0xBB sync, validates CRC, dispatches command to `motor.ref.*`.
4. **STOP/RESUME handling** — `STOP` zeros refs and drives INLx low; `RESUME` restores INLx and re-arms control.
5. **SET_MODE handling** — must call `FOC_Reset()` on every mode change to flush PID integrator state before the new mode runs.
6. **Function generator (`FOC_FuncGen_t`)** — new struct and update function, called each ISR tick to produce the active reference:
   ```c
   typedef struct {
       uint8_t  enabled;
       uint8_t  waveform;  /* 0=step 1=square 2=triangle 3=sine */
       float    frequency; /* Hz */
       float    amplitude; /* peak */
       float    offset;    /* DC bias */
       float    phase;     /* accumulator [0, 1) */
   } FOC_FuncGen_t;
   ```
   Output replaces the static ref for the active mode axis only (e.g. `i_q_ref` in Torque, `omega_ref` in Velocity). Disabled on mode change alongside PID reset.

---

## Work Order

1. MCU: extend telemetry frame
2. MCU: UART RX ring buffer + 7-byte command parser + dispatch
3. Python: serial reader thread — parse frames, feed circular buffer
4. Python: pyqtgraph UI — 6-panel scope + control panel
5. Integration test: verify plots live, then test mode switching and stop/run

---

## Open Questions / Notes

- `theta_mech` in the plot should display the unwrapped multi-turn value (already provided by estimator).
- Angles plot shows `theta_elec mod 2π` for readability (electrical cycles repeat).
- STOP command must be safe to send at any time — MCU must not fault on unexpected stop.
- Per-channel ADC gain trim (Issue 1 — current imbalance) to be addressed separately before closing the current loop.
