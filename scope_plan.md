# FOC Scope Tool — Design Plan

Real-time oscilloscope and control panel for the minimalFOC system, running on the host PC over UART.

---

## Communication Architecture

### MCU → PC (telemetry)

Extend the existing binary frame to include references and active mode.

**Current frame (47 bytes):** 11 × float32 + header + CRC  
**New frame (66 bytes):** 16 × float32 + 1 × uint8 mode + header + CRC

```
[0xAA][0x55][16 × float32 LE = 64 B][uint8 mode][XOR CRC]  — 68 bytes total
```

Fields (in order):
1. `theta_mech`
2. `omega_mech`
3. `theta_elec`
4. `i_u`, `i_v`, `i_w`
5. `i_d`, `i_q`
6. `v_d`, `v_q`
7. `i_d_ref`, `i_q_ref`
8. `v_d_ref`, `v_q_ref`
9. `omega_ref`
10. `theta_ref`
11. `isr_us`

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
┌─────────────────────────────────────────────────────────────┐
│  PORT: [COM3 ▼]  [Connect]   MODE: [Voltage ▼]              │
│  Vd: [0.0]  Vq: [1.0]   [Apply]         [■ STOP]  [▶ RUN]  │
└─────────────────────────────────────────────────────────────┘
┌────────────────────────┐  ┌────────────────────────────────┐
│  UVW Currents (A)      │  │  dq Currents + Refs (A)        │
│  i_u, i_v, i_w         │  │  i_d, i_q, i_d_ref, i_q_ref   │
└────────────────────────┘  └────────────────────────────────┘
┌────────────────────────┐  ┌────────────────────────────────┐
│  dq Voltages (V)       │  │  Angles (rad, mod 2π)          │
│  v_d, v_q              │  │  theta_mech, theta_elec         │
└────────────────────────┘  └────────────────────────────────┘
┌────────────────────────┐  ┌────────────────────────────────┐
│  omega_mech (rad/s)    │  │  ISR time (µs)                 │
└────────────────────────┘  └────────────────────────────────┘
```

Each plot:
- Scrolling 5-second time window
- Toggleable traces (click legend to show/hide)
- Y-axis autoscale

---

## Control Panel Behaviour

- **Mode selector**: Voltage / Torque / Velocity / Position dropdown. On change: sends `SET_MODE`, shows only the relevant reference fields.
- **Reference fields**: float entries. Only active-mode refs shown (e.g. Voltage mode shows Vd/Vq; Torque shows Id_ref/Iq_ref; etc.)
- **Apply**: sends current ref values as individual SET_* commands.
- **STOP**: sends `STOP` command. MCU zeros outputs and disables INLx.
- **RUN**: sends `RESUME`. MCU re-enables switching.

---

## MCU Changes Needed

1. **Extend `log_pack()`** — add refs + mode to telemetry frame, update `LOG_FRAME_BYTES`.
2. **UART RX ring buffer** — small circular buffer fed by USART1 RX DMA (already initialized).
3. **Command parser** — called from main loop; detects 0xBB sync, validates CRC, dispatches command to `motor.ref.*`.
4. **STOP/RESUME handling** — `STOP` zeros refs and drives INLx low; `RESUME` restores INLx and re-arms control.

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
