#!/usr/bin/env python3
"""
ESC Measurement & Calibration View (GUI)

Shows real-time sensor data from LOG_CLASS_CURRENT (2):
  - Phase currents (Iu, Iv, Iw)
  - Clarke outputs (Iα, Iβ)
  - Encoder electrical angle
  - Bus voltage

Buttons:
  Start Log     — sends LOG_CLASS_CURRENT + throttle → motor spins, streams data
  Stop Motor    — stops motor, clears log class
  Calibrate     — sends LOG_CLASS_FOC while IDLE → runs 4-angle calibration
  Reset FC      — hardware reset

Usage:
  python3 measurement_view.py
"""

import serial, serial.tools.list_ports, struct, threading, queue, time, math, sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider

# ── Configuration ──────────────────────────────────────────────────────────

BAUD = 19200
DB_CMD_THROTTLE  = 0x01
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07
LOG_CLASS_FOC     = 1
LOG_CLASS_CURRENT = 2

HISTORY = 300   # samples at 25 Hz = 12 s

# ── Dark theme ─────────────────────────────────────────────────────────────

BG       = '#1e1e1e'
PANEL    = '#252526'
TEXT     = '#cccccc'
DIM      = '#888888'
GRID     = '#3c3c3c'
BTN_GRN  = '#2d5a2d'
BTN_RED  = '#5a2d2d'
BTN_AMB  = '#5a4a2d'

STATE_NAMES = {0:'IDLE', 1:'CALIBRATE', 2:'ALIGN', 3:'RAMP', 4:'OPENLOOP', 5:'CLOSEDLOOP'}
STATE_COLORS = {0:'#888', 1:'#fa0', 2:'#fa0', 3:'#5af', 4:'#5f5', 5:'#f5f'}

# ── Serial port ─────────────────────────────────────────────────────────────

def find_stlink_port():
    fallback = None
    for port, desc, hwid in sorted(serial.tools.list_ports.comports()):
        if '0483' in hwid:
            return port
        if any(x in port for x in ['usbmodem', 'usbserial', 'ttyACM', 'ttyUSB']):
            if not fallback: fallback = port
    return fallback

# ── DB protocol ─────────────────────────────────────────────────────────────

def build_frame(cmd, payload):
    hdr = struct.pack('<2sBBH', b'db', cmd, 0, len(payload))
    ck = cmd + 0 + (len(payload) & 0xFF) + ((len(payload) >> 8) & 0xFF)
    for b in payload: ck += b
    return hdr + payload + struct.pack('<H', ck & 0xFFFF)

class FrameParser:
    S1, S2, CMD, CLS, SL, SH, PL, CL, CH = range(9)
    def __init__(self):
        self.state = self.S1; self.cmd = 0; self.size = 0
        self.payload = bytearray(); self.ck_acc = 0; self.ck_lo = 0
    def feed(self, data):
        for b in data:
            if   self.state == self.S1:
                if b == ord('d'): self.state = self.S2
            elif self.state == self.S2:
                self.state = self.CMD if b == ord('b') else self.S1
            elif self.state == self.CMD:
                self.cmd = b; self.ck_acc = b; self.state = self.CLS
            elif self.state == self.CLS:
                self.ck_acc += b; self.state = self.SL
            elif self.state == self.SL:
                self.size = b; self.ck_acc += b; self.state = self.SH
            elif self.state == self.SH:
                self.size |= b << 8; self.ck_acc += b
                if self.size > 120: self.state = self.S1
                elif self.size == 0: self.state = self.CL
                else: self.payload = bytearray(); self.state = self.PL
            elif self.state == self.PL:
                self.payload.append(b); self.ck_acc += b
                if len(self.payload) >= self.size: self.state = self.CL
            elif self.state == self.CL:
                self.ck_lo = b; self.state = self.CH
            elif self.state == self.CH:
                if (self.ck_lo | (b << 8)) == (self.ck_acc & 0xFFFF):
                    yield (self.cmd, bytes(self.payload))
                self.state = self.S1

# ── Serial reader thread ───────────────────────────────────────────────────

def serial_reader(ser, q, stop_evt):
    parser = FrameParser()
    while not stop_evt.is_set():
        try:
            raw = ser.read(256)
        except Exception:
            break
        if raw:
            for cmd, payload in parser.feed(raw):
                q.put((cmd, payload))

# ── Main ───────────────────────────────────────────────────────────────────

def main():
    port = find_stlink_port()
    if not port:
        print("No serial port found"); sys.exit(1)
    print(f"Port: {port}")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    q = queue.Queue(maxsize=500)
    stop_evt = threading.Event()
    t = threading.Thread(target=serial_reader, args=(ser, q, stop_evt), daemon=True)
    t.start()

    # ── Matplotlib setup ────────────────────────────────────────────────
    plt.rcParams.update({
        'figure.facecolor': BG, 'axes.facecolor': PANEL,
        'axes.edgecolor': GRID, 'axes.labelcolor': TEXT,
        'xtick.color': DIM, 'ytick.color': DIM, 'text.color': TEXT,
        'grid.color': GRID, 'grid.alpha': 0.5,
    })

    fig = plt.figure(figsize=(14, 8))
    fig.canvas.manager.set_window_title('ESC Measurement View')

    # Layout: 3 rows of plots + button bar
    gs = fig.add_gridspec(4, 2, hspace=0.35, wspace=0.25,
                          left=0.06, right=0.97, top=0.95, bottom=0.12)

    ax_phase  = fig.add_subplot(gs[0, 0])  # Phase currents
    ax_clarke = fig.add_subplot(gs[0, 1])  # Clarke Iα, Iβ
    ax_enc    = fig.add_subplot(gs[1, 0])  # Encoder angle
    ax_vbus   = fig.add_subplot(gs[1, 1])  # Bus voltage
    ax_liss   = fig.add_subplot(gs[2, 0])  # Lissajous Iα vs Iβ
    ax_cal    = fig.add_subplot(gs[2, 1])  # Calibration status

    for ax in [ax_phase, ax_clarke, ax_enc, ax_vbus]:
        ax.set_xlim(0, HISTORY)
        ax.grid(True, linewidth=0.5)

    ax_phase.set_title('Phase Currents', fontsize=10)
    ax_phase.set_ylabel('A')
    line_iu, = ax_phase.plot([], [], color='#ff6b6b', linewidth=1, label='Iu')
    line_iv, = ax_phase.plot([], [], color='#6bff6b', linewidth=1, label='Iv')
    line_iw, = ax_phase.plot([], [], color='#6b6bff', linewidth=1, label='Iw')
    ax_phase.legend(fontsize=7, loc='upper right')

    ax_clarke.set_title('Clarke (Iα, Iβ)', fontsize=10)
    ax_clarke.set_ylabel('A')
    line_ia, = ax_clarke.plot([], [], color='#ffaa55', linewidth=1, label='Iα')
    line_ib, = ax_clarke.plot([], [], color='#55aaff', linewidth=1, label='Iβ')
    ax_clarke.legend(fontsize=7, loc='upper right')

    ax_enc.set_title('Encoder Electrical Angle', fontsize=10)
    ax_enc.set_ylabel('deg')
    ax_enc.set_ylim(-10, 370)
    line_enc, = ax_enc.plot([], [], color='#55ff55', linewidth=1)

    ax_vbus.set_title('Bus Voltage', fontsize=10)
    ax_vbus.set_ylabel('V')
    line_vbus, = ax_vbus.plot([], [], color='#ff55ff', linewidth=1)

    ax_liss.set_title('Lissajous (Iα vs Iβ)', fontsize=10)
    ax_liss.set_xlabel('Iα (A)')
    ax_liss.set_ylabel('Iβ (A)')
    ax_liss.set_aspect('equal')
    ax_liss.grid(True, linewidth=0.5)
    line_liss, = ax_liss.plot([], [], color='#55aaff', linewidth=0.8, alpha=0.7)
    liss_dot, = ax_liss.plot([], [], 'o', color='#ff5555', markersize=4)

    ax_cal.set_title('Status', fontsize=10)
    ax_cal.set_xlim(0, 1); ax_cal.set_ylim(0, 1)
    ax_cal.axis('off')
    status_text = ax_cal.text(0.5, 0.5, 'IDLE', fontsize=16, ha='center', va='center',
                              color=STATE_COLORS[0], fontweight='bold')
    info_text = ax_cal.text(0.5, 0.15, '', fontsize=9, ha='center', va='center', color=DIM)

    # Data buffers
    h_iu = np.zeros(HISTORY)
    h_iv = np.zeros(HISTORY)
    h_iw = np.zeros(HISTORY)
    h_ia = np.zeros(HISTORY)
    h_ib = np.zeros(HISTORY)
    h_enc = np.zeros(HISTORY)
    h_vbus = np.zeros(HISTORY)
    x = np.arange(HISTORY)
    last_state = [0]
    cal_result = ['']

    # ── Buttons & slider ────────────────────────────────────────────────
    ax_start = fig.add_axes([0.06, 0.02, 0.12, 0.04])
    ax_stop  = fig.add_axes([0.20, 0.02, 0.12, 0.04])
    ax_cal_b = fig.add_axes([0.34, 0.02, 0.12, 0.04])
    ax_reset = fig.add_axes([0.48, 0.02, 0.12, 0.04])
    ax_thr   = fig.add_axes([0.66, 0.03, 0.28, 0.02])

    btn_start = Button(ax_start, 'Start Log', color=BTN_GRN, hovercolor='#3d7a3d')
    btn_stop  = Button(ax_stop,  'Stop Motor', color=BTN_RED, hovercolor='#7a3d3d')
    btn_cal   = Button(ax_cal_b, 'Calibrate', color=BTN_AMB, hovercolor='#7a6a3d')
    btn_reset = Button(ax_reset, 'Reset FC', color=BTN_RED, hovercolor='#7a3d3d')
    slider_thr = Slider(ax_thr, 'Throttle', 0.0, 0.30, valinit=0.05, color='#55aaff')

    for b in [btn_start, btn_stop, btn_cal, btn_reset]:
        b.label.set_color(TEXT)
        b.label.set_fontsize(9)

    def on_start(evt):
        ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([LOG_CLASS_CURRENT])))
        ser.flush()
        time.sleep(0.1)
        thr = slider_thr.val
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', thr)))
        ser.flush()

    def on_stop(evt):
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0)))
        ser.flush()

    def on_calibrate(evt):
        # Must be IDLE. Send LOG_CLASS_FOC to trigger calibration.
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0)))
        ser.flush()
        time.sleep(0.3)
        cal_result[0] = 'Running...'
        ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([LOG_CLASS_FOC])))
        ser.flush()

    def on_reset(evt):
        send_reset_frame(ser)

    def send_reset_frame(s):
        s.write(build_frame(DB_CMD_RESET, b''))
        s.flush()

    btn_start.on_clicked(on_start)
    btn_stop.on_clicked(on_stop)
    btn_cal.on_clicked(on_calibrate)
    btn_reset.on_clicked(on_reset)

    def on_throttle(val):
        if last_state[0] >= 2:  # motor running
            ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', val)))
            ser.flush()

    slider_thr.on_changed(on_throttle)

    # ── Animation ───────────────────────────────────────────────────────
    def update(frame_num):
        nonlocal h_iu, h_iv, h_iw, h_ia, h_ib, h_enc, h_vbus

        count = 0
        while not q.empty() and count < 50:
            try:
                cmd, payload = q.get_nowait()
            except queue.Empty:
                break
            count += 1

            if cmd == 0x00 and len(payload) == 32:
                f = struct.unpack('<8f', payload)
                state = int(f[0])
                last_state[0] = state

                # LOG_CLASS_CURRENT format:
                # [0]state [1]Iα [2]Iβ [3]enc_elec [4]Iu [5]Iv [6]Iw [7]Vbus
                h_ia  = np.roll(h_ia,  -1); h_ia[-1]  = f[1]
                h_ib  = np.roll(h_ib,  -1); h_ib[-1]  = f[2]
                h_enc = np.roll(h_enc, -1); h_enc[-1] = math.degrees(f[3]) % 360
                h_iu  = np.roll(h_iu,  -1); h_iu[-1]  = f[4]
                h_iv  = np.roll(h_iv,  -1); h_iv[-1]  = f[5]
                h_iw  = np.roll(h_iw,  -1); h_iw[-1]  = f[6]
                h_vbus = np.roll(h_vbus,-1); h_vbus[-1] = f[7]

                # Update status
                sname = STATE_NAMES.get(state, f'?{state}')
                scolor = STATE_COLORS.get(state, '#888')
                status_text.set_text(sname)
                status_text.set_color(scolor)

                # Calibration result tracking
                if state == 1:  # CALIBRATE
                    cal_result[0] = f'Calibrating step {int(f[1]) + 1}/4...' if cmd == 0x00 else cal_result[0]
                elif state == 0 and cal_result[0].startswith(('Running', 'Calibrating')):
                    cal_result[0] = 'Done'

            elif cmd == 0xFF:
                pass  # heartbeat

        # Update plots
        line_iu.set_data(x, h_iu)
        line_iv.set_data(x, h_iv)
        line_iw.set_data(x, h_iw)
        ax_phase.relim(); ax_phase.autoscale_view(scalex=False)

        line_ia.set_data(x, h_ia)
        line_ib.set_data(x, h_ib)
        ax_clarke.relim(); ax_clarke.autoscale_view(scalex=False)

        line_enc.set_data(x, h_enc)

        line_vbus.set_data(x, h_vbus)
        ax_vbus.relim(); ax_vbus.autoscale_view(scalex=False)

        # Lissajous — last 100 samples
        n = 100
        line_liss.set_data(h_ia[-n:], h_ib[-n:])
        if h_ia[-1] != 0 or h_ib[-1] != 0:
            liss_dot.set_data([h_ia[-1]], [h_ib[-1]])
        ax_liss.relim(); ax_liss.autoscale_view()

        info_text.set_text(
            f'Vbus={h_vbus[-1]:.1f}V  Enc={h_enc[-1]:.0f}°\n{cal_result[0]}'
        )

        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=80)
    timer.add_callback(update, 0)
    timer.start()

    def on_close(evt):
        stop_evt.set()
        try:
            ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0)))
            ser.flush()
        except Exception:
            pass
        ser.close()

    fig.canvas.mpl_connect('close_event', on_close)
    plt.show()


if __name__ == '__main__':
    main()
