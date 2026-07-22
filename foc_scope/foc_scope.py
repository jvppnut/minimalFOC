#!/usr/bin/env python3
"""
foc_scope.py — Real-time oscilloscope for minimalFOC over UART.

Telemetry frame (MCU → PC), 64 bytes:
  [0xAA][0x55][15 × float32 LE = 60 B][uint8 mode][XOR CRC]
  Fields: theta_mech, omega_mech, theta_elec,
          i_u, i_v, i_w, i_d, i_q, v_d, v_q,
          i_d_ref, i_q_ref, omega_ref, theta_ref, isr_us

Command frame (PC → MCU), 7 bytes:
  [0xBB][CMD][float32 LE = 4 B][XOR CRC]
  CRC = XOR of bytes 1–5 (CMD + payload)
"""

import sys
import csv
import struct
import serial
import threading
import queue
import collections
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QComboBox, QPushButton, QLineEdit, QGroupBox,
    QFileDialog,
)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg

# ── Frame / field constants ────────────────────────────────────────────────────
FRAME_BYTES  = 64
FRAME_HEADER = bytes([0xAA, 0x55])
N_FLOATS     = 15
FIELDS = [
    'theta_mech', 'omega_mech', 'theta_elec',
    'i_u', 'i_v', 'i_w',
    'i_d', 'i_q',
    'v_d', 'v_q',
    'i_d_ref', 'i_q_ref',
    'omega_ref', 'theta_ref',
    'isr_us',
]
IDX = {f: i for i, f in enumerate(FIELDS)}

# ── Command codes ──────────────────────────────────────────────────────────────
CMD_SET_MODE      = 0x01
CMD_SET_VD_REF    = 0x02
CMD_SET_VQ_REF    = 0x03
CMD_SET_ID_REF    = 0x04
CMD_SET_IQ_REF    = 0x05
CMD_SET_OMEGA_REF = 0x06
CMD_SET_THETA_REF = 0x07
CMD_STOP          = 0x08
CMD_RESUME        = 0x09
CMD_FGEN_WAVE     = 0x0A
CMD_FGEN_FREQ     = 0x0B
CMD_FGEN_AMP      = 0x0C
CMD_FGEN_OFFSET   = 0x0D
CMD_FGEN_ENABLE   = 0x0E

MODES     = ['Voltage', 'Torque', 'Velocity', 'Position']
WAVEFORMS = ['Step', 'Square', 'Triangle', 'Sine', 'Staircase2']
BAUD      = 921600
SAMPLE_RATE = 1000  # Hz (20 kHz ISR decimated ×20)

# Ref keys active per mode — only these fields are visible in the control bar
MODE_REF_KEYS = {
    'Voltage':  ['vd', 'vq'],
    'Torque':   ['id', 'iq'],
    'Velocity': ['omega'],
    'Position': ['theta'],
}
REF_CMD = {
    'vd':    CMD_SET_VD_REF,
    'vq':    CMD_SET_VQ_REF,
    'id':    CMD_SET_ID_REF,
    'iq':    CMD_SET_IQ_REF,
    'omega': CMD_SET_OMEGA_REF,
    'theta': CMD_SET_THETA_REF,
}


def build_cmd(cmd_byte: int, value: float) -> bytes:
    """Pack a 7-byte command frame with XOR CRC covering bytes 1–5."""
    payload = struct.pack('<f', value)
    body = bytes([cmd_byte]) + payload          # bytes 1–5
    crc = 0
    for b in body:
        crc ^= b
    return bytes([0xBB]) + body + bytes([crc])  # 7 bytes


# ── Serial reader thread ───────────────────────────────────────────────────────
class SerialReader(threading.Thread):
    """Reads raw UART bytes, parses 64-byte telemetry frames, queues (floats, mode)."""

    def __init__(self, port: str, baud: int, data_queue: queue.Queue):
        super().__init__(daemon=True)
        self.port  = port
        self.baud  = baud
        self.q     = data_queue
        self._stop = threading.Event()
        self._ser  = None
        self.error: str | None = None

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except serial.SerialException as e:
            self.error = str(e)
            return

        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            # Parse as many complete frames as possible
            while True:
                idx = buf.find(FRAME_HEADER)
                if idx < 0:
                    buf = buf[-1:]   # keep last byte — might be 0xAA
                    break
                if idx > 0:
                    buf = buf[idx:]  # discard leading garbage
                if len(buf) < FRAME_BYTES:
                    break
                frame = buf[:FRAME_BYTES]
                # Validate CRC (bytes 2–62)
                crc = 0
                for b in frame[2:63]:
                    crc ^= b
                if crc == frame[63]:
                    floats = struct.unpack_from('<15f', frame, 2)
                    mode   = frame[62]
                    try:
                        self.q.put_nowait((floats, mode))
                    except queue.Full:
                        pass
                    buf = buf[FRAME_BYTES:]
                else:
                    buf = buf[1:]   # bad CRC — advance 1 byte and resync

        if self._ser and self._ser.is_open:
            self._ser.close()

    def stop(self):
        self._stop.set()

    def send(self, data: bytes):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(data)
            except serial.SerialException:
                pass


# ── Main window ────────────────────────────────────────────────────────────────
class FocScope(QMainWindow):
    WINDOW_OPTIONS  = [0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0]
    DEFAULT_WINDOW  = 1.0
    UPDATE_MS       = 33   # ~30 fps
    MAX_BUF_SAMPLES = int(20 * SAMPLE_RATE)  # 20 s at full rate

    # Trace colour palette
    COLORS = {
        'i_u':      '#e74c3c',
        'i_v':      '#2ecc71',
        'i_w':      '#3498db',
        'i_d':      '#e74c3c',
        'i_q':      '#2ecc71',
        'i_d_ref':  '#ff8c69',
        'i_q_ref':  '#90ee90',
        'v_d':      '#e74c3c',
        'v_q':      '#2ecc71',
        'theta_mech': '#f1c40f',
        'theta_elec': '#9b59b6',
        'omega_mech': '#1abc9c',
        'isr_us':   '#f39c12',
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle('FOC Scope')
        self.resize(1440, 920)

        self._reader: SerialReader | None = None
        self._queue  = queue.Queue(maxsize=8192)

        self._window_s = self.DEFAULT_WINDOW
        self._max_pts  = int(self._window_s * SAMPLE_RATE)

        self._t_buf = collections.deque(maxlen=self.MAX_BUF_SAMPLES)
        self._bufs  = {f: collections.deque(maxlen=self.MAX_BUF_SAMPLES)
                       for f in FIELDS}
        self._t_now = 0.0
        self._rx_mode = 0
        self._frozen = False

        self._build_ui()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update)
        self._timer.start(self.UPDATE_MS)

    # ── UI construction ────────────────────────────────────────────────────────

    def _build_ui(self):
        pg.setConfigOptions(antialias=True, foreground='w', background='k')
        root = QWidget()
        self.setCentralWidget(root)
        vbox = QVBoxLayout(root)
        vbox.setContentsMargins(4, 4, 4, 4)
        vbox.setSpacing(3)

        vbox.addWidget(self._build_control_bar())
        vbox.addWidget(self._build_fgen_bar())
        vbox.addWidget(self._build_display_bar())

        vbox.addWidget(self._build_plots(), stretch=1)

        vbox.addWidget(self._build_live_panel())

    def _build_control_bar(self) -> QGroupBox:
        bar = QGroupBox()
        h = QHBoxLayout(bar)
        h.setSpacing(6)

        # Connection
        h.addWidget(QLabel('Port:'))
        self._port_edit = QLineEdit('COM3')
        self._port_edit.setFixedWidth(64)
        h.addWidget(self._port_edit)
        self._connect_btn = QPushButton('Connect')
        self._connect_btn.clicked.connect(self._on_connect)
        h.addWidget(self._connect_btn)
        h.addWidget(_sep())

        # Mode
        h.addWidget(QLabel('Mode:'))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(MODES)
        h.addWidget(self._mode_combo)
        h.addWidget(_sep())

        # Time window
        h.addWidget(QLabel('Window:'))
        self._win_combo = QComboBox()
        for w in self.WINDOW_OPTIONS:
            self._win_combo.addItem(f'{w} s')
        self._win_combo.setCurrentIndex(self.WINDOW_OPTIONS.index(self.DEFAULT_WINDOW))
        self._win_combo.currentIndexChanged.connect(self._on_window_change)
        h.addWidget(self._win_combo)
        h.addWidget(_sep())

        # Reference inputs — each is (container_widget, QLineEdit)
        self._ref_containers: dict[str, QWidget] = {}
        self._ref_edits: dict[str, QLineEdit] = {}
        ref_defs = [
            ('vd',    'Vd (V):',     '0.0'),
            ('vq',    'Vq (V):',     '0.0'),
            ('id',    'Id_ref (A):', '0.0'),
            ('iq',    'Iq_ref (A):', '0.0'),
            ('omega', 'ω_ref (r/s):', '0.0'),
            ('theta', 'θ_ref (rad):', '0.0'),
        ]
        for key, label, default in ref_defs:
            c = QWidget()
            cl = QHBoxLayout(c)
            cl.setContentsMargins(0, 0, 0, 0)
            cl.setSpacing(3)
            cl.addWidget(QLabel(label))
            ed = QLineEdit(default)
            ed.setFixedWidth(58)
            cl.addWidget(ed)
            self._ref_containers[key] = c
            self._ref_edits[key] = ed
            h.addWidget(c)

        self._apply_btn = QPushButton('Apply')
        self._apply_btn.clicked.connect(self._on_apply)
        h.addWidget(self._apply_btn)
        h.addStretch()

        self._stop_btn = QPushButton('■  STOP')
        self._stop_btn.setStyleSheet(
            'QPushButton { background: #c0392b; color: white; font-weight: bold; padding: 4px 10px; }')
        self._stop_btn.clicked.connect(self._on_stop)
        h.addWidget(self._stop_btn)

        self._run_btn = QPushButton('▶  RUN')
        self._run_btn.setStyleSheet(
            'QPushButton { background: #27ae60; color: white; font-weight: bold; padding: 4px 10px; }')
        self._run_btn.clicked.connect(self._on_run)
        h.addWidget(self._run_btn)

        # Wire mode combo AFTER widget creation to avoid spurious sends on startup
        self._mode_combo.currentIndexChanged.connect(self._on_mode_change)
        self._update_ref_visibility('Voltage')
        return bar

    def _build_fgen_bar(self) -> QGroupBox:
        bar = QGroupBox('Function Generator')
        h = QHBoxLayout(bar)
        h.setSpacing(6)

        h.addWidget(QLabel('Wave:'))
        self._fgen_wave = QComboBox()
        self._fgen_wave.addItems(WAVEFORMS)
        self._fgen_wave.setCurrentIndex(3)   # Sine default
        h.addWidget(self._fgen_wave)

        h.addWidget(QLabel('Freq (Hz):'))
        self._fgen_freq = QLineEdit('1.0')
        self._fgen_freq.setFixedWidth(60)
        h.addWidget(self._fgen_freq)

        h.addWidget(QLabel('Amp:'))
        self._fgen_amp = QLineEdit('0.5')
        self._fgen_amp.setFixedWidth(60)
        h.addWidget(self._fgen_amp)

        h.addWidget(QLabel('Offset:'))
        self._fgen_off = QLineEdit('0.0')
        self._fgen_off.setFixedWidth(60)
        h.addWidget(self._fgen_off)

        set_btn = QPushButton('Set')
        set_btn.clicked.connect(self._on_fgen_set)
        h.addWidget(set_btn)
        h.addStretch()

        self._fgen_enable_btn = QPushButton('Enable Fgen')
        self._fgen_enable_btn.setCheckable(True)
        self._fgen_enable_btn.setStyleSheet(
            'QPushButton:checked { background: #e67e22; color: white; font-weight: bold; }')
        self._fgen_enable_btn.toggled.connect(self._on_fgen_enable)
        h.addWidget(self._fgen_enable_btn)

        return bar

    def _build_display_bar(self) -> QGroupBox:
        bar = QGroupBox()
        h = QHBoxLayout(bar)
        h.setSpacing(6)

        self._freeze_btn = QPushButton('❄  Freeze')
        self._freeze_btn.setCheckable(True)
        self._freeze_btn.setStyleSheet(
            'QPushButton:checked { background: #2980b9; color: white; font-weight: bold; }')
        self._freeze_btn.toggled.connect(self._on_freeze_toggle)
        h.addWidget(self._freeze_btn)

        self._save_csv_btn = QPushButton('Save CSV')
        self._save_csv_btn.setEnabled(False)
        self._save_csv_btn.clicked.connect(self._on_save_csv)
        h.addWidget(self._save_csv_btn)
        h.addWidget(_sep())

        h.addWidget(QLabel('Y-Axis:'))
        self._yaxis_mode_combo = QComboBox()
        self._yaxis_mode_combo.addItems(['Auto', 'Manual'])
        self._yaxis_mode_combo.currentIndexChanged.connect(self._on_yaxis_mode_change)
        h.addWidget(self._yaxis_mode_combo)
        h.addStretch()

        return bar

    def _build_plots(self) -> QWidget:
        container = QWidget()
        grid = QGridLayout(container)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(4)

        self._curves: dict[str, pg.PlotDataItem] = {}
        self._plot_widgets: dict[str, pg.PlotWidget] = {}
        self._yaxis_edits: dict[str, tuple[QLineEdit, QLineEdit]] = {}

        def add_plot(row, col, key, title, ylabel, traces, y_default):
            """traces: list of (field_key, legend_label)"""
            pw = pg.PlotWidget(title=title)
            pw.setLabel('left', ylabel)
            pw.setLabel('bottom', 't (s)')
            pw.addLegend(offset=(5, 5))
            pw.showGrid(x=True, y=True, alpha=0.25)
            for f, lbl in traces:
                pen = pg.mkPen(color=self.COLORS.get(f, '#ffffff'), width=1)
                self._curves[f] = pw.plot(pen=pen, name=lbl)
            self._plot_widgets[key] = pw

            cell = QWidget()
            vb = QVBoxLayout(cell)
            vb.setContentsMargins(0, 0, 0, 0)
            vb.setSpacing(2)
            vb.addWidget(pw, stretch=1)

            yr = QHBoxLayout()
            yr.setContentsMargins(0, 0, 0, 0)
            yr.setSpacing(3)
            yr.addStretch()
            yr.addWidget(QLabel('Y min:'))
            ymin_edit = QLineEdit(str(y_default[0]))
            ymin_edit.setFixedWidth(55)
            yr.addWidget(ymin_edit)
            yr.addWidget(QLabel('Y max:'))
            ymax_edit = QLineEdit(str(y_default[1]))
            ymax_edit.setFixedWidth(55)
            yr.addWidget(ymax_edit)
            yr.addStretch()
            vb.addLayout(yr)

            ymin_edit.editingFinished.connect(lambda k=key: self._on_yrange_edit(k))
            ymax_edit.editingFinished.connect(lambda k=key: self._on_yrange_edit(k))
            self._yaxis_edits[key] = (ymin_edit, ymax_edit)

            grid.addWidget(cell, row, col)

        add_plot(0, 0, 'uvw', 'UVW Currents', 'A',
                 [('i_u','i_u'), ('i_v','i_v'), ('i_w','i_w')], (-10.0, 10.0))
        add_plot(0, 1, 'dq_i', 'dq Currents + Refs', 'A',
                 [('i_d','i_d'), ('i_q','i_q'),
                  ('i_d_ref','i_d_ref'), ('i_q_ref','i_q_ref')], (-10.0, 10.0))
        add_plot(1, 0, 'dq_v', 'dq Voltages (post-limiter)', 'V',
                 [('v_d','v_d'), ('v_q','v_q')], (-12.0, 12.0))
        add_plot(1, 1, 'angles', 'Angles', 'rad',
                 [('theta_mech','θ_mech'), ('theta_elec','θ_elec')], (-3.5, 3.5))
        add_plot(2, 0, 'omega', 'Mechanical Velocity', 'rad/s',
                 [('omega_mech','ω_mech')], (-50.0, 50.0))
        add_plot(2, 1, 'isr', 'ISR Time', 'µs',
                 [('isr_us','isr')], (0.0, 50.0))

        for r in range(3):
            grid.setRowStretch(r, 1)
        for c in range(2):
            grid.setColumnStretch(c, 1)

        return container

    def _build_live_panel(self) -> QGroupBox:
        box = QGroupBox('Live Values')
        h = QHBoxLayout(box)
        h.setSpacing(0)
        self._live_labels: dict[str, QLabel] = {}

        live_order = FIELDS + ['mode']
        live_names = {
            'theta_mech': 'θ_mech', 'omega_mech': 'ω_mech',
            'theta_elec': 'θ_elec', 'i_u': 'i_u', 'i_v': 'i_v',
            'i_w': 'i_w', 'i_d': 'i_d', 'i_q': 'i_q',
            'v_d': 'v_d', 'v_q': 'v_q', 'i_d_ref': 'i_d_ref',
            'i_q_ref': 'i_q_ref', 'omega_ref': 'ω_ref',
            'theta_ref': 'θ_ref', 'isr_us': 'isr(µs)', 'mode': 'mode',
        }

        for f in live_order:
            cell = QWidget()
            vl = QVBoxLayout(cell)
            vl.setContentsMargins(4, 1, 4, 1)
            vl.setSpacing(0)

            lname = QLabel(live_names.get(f, f))
            lname.setAlignment(Qt.AlignCenter)
            lname.setStyleSheet('color: #888; font-size: 8pt;')

            lval = QLabel('—')
            lval.setAlignment(Qt.AlignCenter)
            lval.setStyleSheet('font-weight: bold; font-size: 9pt;')
            lval.setMinimumWidth(70)

            vl.addWidget(lname)
            vl.addWidget(lval)
            h.addWidget(cell)
            self._live_labels[f] = lval

        return box

    # ── Slot handlers ──────────────────────────────────────────────────────────

    def _on_connect(self):
        if self._reader and self._reader.is_alive():
            self._reader.stop()
            self._connect_btn.setText('Connect')
            self._connect_btn.setStyleSheet('')
            return
        port = self._port_edit.text().strip()
        self._queue = queue.Queue(maxsize=8192)
        self._reader = SerialReader(port, BAUD, self._queue)
        self._reader.start()
        self._connect_btn.setText('Disconnect')
        self._connect_btn.setStyleSheet('background: #2980b9; color: white;')

    def _on_mode_change(self, idx: int):
        mode_name = MODES[idx]
        self._zero_and_stop(new_mode=idx)
        self._update_ref_visibility(mode_name)

    def _zero_and_stop(self, new_mode: int | None = None):
        """Send STOP, zero all UI refs, disable fgen, optionally switch mode."""
        self._send(CMD_STOP, 0.0)
        for ed in self._ref_edits.values():
            ed.setText('0.0')
        self._fgen_enable_btn.setChecked(False)  # triggers _on_fgen_enable(False)
        if new_mode is not None:
            self._send(CMD_SET_MODE, float(new_mode))

    def _update_ref_visibility(self, mode_name: str):
        active = MODE_REF_KEYS.get(mode_name, [])
        for key, c in self._ref_containers.items():
            c.setVisible(key in active)

    def _on_apply(self):
        mode_name = MODES[self._mode_combo.currentIndex()]
        for key in MODE_REF_KEYS.get(mode_name, []):
            try:
                val = float(self._ref_edits[key].text())
                self._send(REF_CMD[key], val)
            except (ValueError, KeyError):
                pass

    def _on_stop(self):
        self._send(CMD_STOP, 0.0)
        self._fgen_enable_btn.setChecked(False)

    def _on_run(self):
        self._send(CMD_RESUME, 0.0)

    def _on_fgen_set(self):
        try:
            self._send(CMD_FGEN_WAVE,   float(self._fgen_wave.currentIndex()))
            self._send(CMD_FGEN_FREQ,   float(self._fgen_freq.text()))
            self._send(CMD_FGEN_AMP,    float(self._fgen_amp.text()))
            self._send(CMD_FGEN_OFFSET, float(self._fgen_off.text()))
        except ValueError:
            pass

    def _on_fgen_enable(self, checked: bool):
        self._send(CMD_FGEN_ENABLE, 1.0 if checked else 0.0)

    def _on_window_change(self, idx: int):
        self._window_s = self.WINDOW_OPTIONS[idx]
        self._max_pts  = int(self._window_s * SAMPLE_RATE)

    def _on_freeze_toggle(self, checked: bool):
        self._frozen = checked
        self._save_csv_btn.setEnabled(checked)
        self._freeze_btn.setText('▶  Live' if checked else '❄  Freeze')
        if not checked:
            # Discard whatever queued up while frozen — resume from "now"
            # rather than fast-forwarding through a stale backlog.
            while True:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break

    def _on_save_csv(self):
        if not self._t_buf:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, 'Save CSV', 'foc_log.csv', 'CSV Files (*.csv)')
        if not path:
            return
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t'] + FIELDS)
            writer.writerows(zip(self._t_buf, *(self._bufs[f] for f in FIELDS)))

    def _on_yaxis_mode_change(self, idx: int):
        manual = (idx == 1)
        for key, pw in self._plot_widgets.items():
            if manual:
                self._apply_yrange(key)
            else:
                pw.enableAutoRange(axis='y', enable=True)

    def _apply_yrange(self, key: str):
        ymin_edit, ymax_edit = self._yaxis_edits[key]
        try:
            ymin = float(ymin_edit.text())
            ymax = float(ymax_edit.text())
        except ValueError:
            return
        self._plot_widgets[key].setYRange(ymin, ymax, padding=0)

    def _on_yrange_edit(self, key: str):
        if self._yaxis_mode_combo.currentIndex() == 1:
            self._apply_yrange(key)

    def _send(self, cmd_byte: int, value: float):
        if self._reader and self._reader.is_alive():
            self._reader.send(build_cmd(cmd_byte, value))

    # ── Plot / live-values update (called at ~30 Hz) ───────────────────────────

    def _update(self):
        if self._frozen:
            return

        # Drain the queue — process up to 300 frames per tick to avoid falling behind
        n = 0
        while n < 300:
            try:
                floats, mode = self._queue.get_nowait()
            except queue.Empty:
                break
            self._t_now += 1.0 / SAMPLE_RATE
            self._t_buf.append(self._t_now)
            for f in FIELDS:
                self._bufs[f].append(floats[IDX[f]])
            self._rx_mode = mode
            n += 1

        if not self._t_buf:
            return

        mp = self._max_pts
        t_arr = np.array(self._t_buf)[-mp:]

        for f, curve in self._curves.items():
            curve.setData(t_arr, np.array(self._bufs[f])[-mp:])

        # Live values panel
        mode_names = ['Voltage', 'Torque', 'Velocity', 'Position', 'Cal']
        for f in FIELDS:
            if not self._bufs[f]:
                continue
            v = self._bufs[f][-1]
            if f == 'isr_us':
                txt = f'{v:.2f}'
            elif f in ('theta_mech', 'theta_elec', 'omega_mech',
                       'omega_ref', 'theta_ref'):
                txt = f'{v:.3f}'
            else:
                txt = f'{v:.4f}'
            self._live_labels[f].setText(txt)

        m = self._rx_mode
        self._live_labels['mode'].setText(
            mode_names[m] if m < len(mode_names) else str(m))

    def closeEvent(self, event):
        if self._reader:
            self._reader.stop()
        super().closeEvent(event)


# ── Helper ─────────────────────────────────────────────────────────────────────
def _sep() -> QWidget:
    """Thin vertical separator line."""
    w = QWidget()
    w.setFixedWidth(1)
    w.setFixedHeight(20)
    w.setStyleSheet('background: #444;')
    return w


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = FocScope()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
