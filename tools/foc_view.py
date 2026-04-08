#!/usr/bin/env python3
"""
ESC FOC Visualization (GUI)

Two modes:
  Open-Loop  — LOG_CLASS_VOLTAGE (3): Id, Iq, θ, Iα, Iβ
  Closed-Loop — LOG_CLASS_ENCODER (5): Id, Iq, Vd, Vq, Iq_ref, θ

Buttons:
  Open-Loop     — calibrate → spin in open-loop, show Park outputs
  Closed-Loop   — calibrate → spin in closed-loop, show PI outputs
  Stop Motor    — stop motor
  Reset FC      — hardware reset

Usage:
  python3 foc_view.py
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

LOG_CLASS_FOC      = 1  # triggers calibration
LOG_CLASS_VOLTAGE  = 3  # open-loop: Park outputs
LOG_CLASS_ENCODER  = 5  # closed-loop: current loop

HISTORY = 300

# ── Dark theme ─────────────────────────────────────────────────────────────

BG       = '#1e1e1e'
PANEL    = '#252526'
TEXT     = '#cccccc'
DIM      = '#888888'
GRID     = '#3c3c3c'
BTN_GRN  = '#2d5a2d'
BTN_RED  = '#5a2d2d'
BTN_AMB  = '#5a4a2d'
BTN_BLU  = '#2d4a5a'

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

    fig = plt.figure(figsize=(14, 9))
    fig.canvas.manager.set_window_title('ESC FOC View')

    gs = fig.add_gridspec(3, 2, hspace=0.35, wspace=0.25,
                          left=0.06, right=0.97, top=0.95, bottom=0.12)

    ax_idiq   = fig.add_subplot(gs[0, 0])  # Id, Iq time series
    ax_clarke = fig.add_subplot(gs[0, 1])  # Iα, Iβ time series
    ax_theta  = fig.add_subplot(gs[1, 0])  # Rotor angle
    ax_vdvq   = fig.add_subplot(gs[1, 1])  # Vd, Vq (closed-loop)
    ax_liss   = fig.add_subplot(gs[2, 0])  # Lissajous Iα vs Iβ
    ax_status = fig.add_subplot(gs[2, 1])  # Status panel

    for ax in [ax_idiq, ax_clarke, ax_theta, ax_vdvq]:
        ax.set_xlim(0, HISTORY)
        ax.grid(True, linewidth=0.5)

    ax_idiq.set_title('Park Outputs (Id, Iq)', fontsize=10)
    ax_idiq.set_ylabel('A')
    line_id, = ax_idiq.plot([], [], color='#ff6b6b', linewidth=1, label='Id')
    line_iq, = ax_idiq.plot([], [], color='#6bff6b', linewidth=1, label='Iq')
    line_iqref, = ax_idiq.plot([], [], color='#6bff6b', linewidth=1, linestyle='--',
                                alpha=0.5, label='Iq_ref')
    ax_idiq.legend(fontsize=7, loc='upper right')

    ax_clarke.set_title('Clarke Outputs (Iα, Iβ)', fontsize=10)
    ax_clarke.set_ylabel('A')
    line_ia, = ax_clarke.plot([], [], color='#ffaa55', linewidth=1, label='Iα')
    line_ib, = ax_clarke.plot([], [], color='#55aaff', linewidth=1, label='Iβ')
    ax_clarke.legend(fontsize=7, loc='upper right')

    ax_theta.set_title('Rotor Angle', fontsize=10)
    ax_theta.set_ylabel('deg')
    ax_theta.set_ylim(-10, 370)
    line_theta, = ax_theta.plot([], [], color='#55ff55', linewidth=1)

    ax_vdvq.set_title('PI Outputs (Vd, Vq)', fontsize=10)
    ax_vdvq.set_ylabel('V')
    line_vd, = ax_vdvq.plot([], [], color='#ff6b6b', linewidth=1, label='Vd')
    line_vq, = ax_vdvq.plot([], [], color='#6bff6b', linewidth=1, label='Vq')
    ax_vdvq.legend(fontsize=7, loc='upper right')

    ax_liss.set_title('Lissajous (Iα vs Iβ)', fontsize=10)
    ax_liss.set_xlabel('Iα (A)'); ax_liss.set_ylabel('Iβ (A)')
    ax_liss.set_aspect('equal')
    ax_liss.grid(True, linewidth=0.5)
    line_liss, = ax_liss.plot([], [], color='#55aaff', linewidth=0.8, alpha=0.7)

    ax_status.set_xlim(0, 1); ax_status.set_ylim(0, 1)
    ax_status.axis('off')
    status_text = ax_status.text(0.5, 0.6, 'IDLE', fontsize=16, ha='center',
                                  va='center', fontweight='bold', color='#888')
    mode_text = ax_status.text(0.5, 0.35, '', fontsize=11, ha='center',
                                va='center', color='#5af')
    info_text = ax_status.text(0.5, 0.1, '', fontsize=9, ha='center',
                                va='center', color=DIM)

    # Data buffers
    h_id = np.zeros(HISTORY)
    h_iq = np.zeros(HISTORY)
    h_iqref = np.zeros(HISTORY)
    h_ia = np.zeros(HISTORY)
    h_ib = np.zeros(HISTORY)
    h_theta = np.zeros(HISTORY)
    h_vd = np.zeros(HISTORY)
    h_vq = np.zeros(HISTORY)
    x = np.arange(HISTORY)

    last_state = [0]
    mode = ['none']  # 'openloop' or 'closedloop'

    # ── Buttons & slider ────────────────────────────────────────────────
    ax_ol   = fig.add_axes([0.06, 0.02, 0.12, 0.04])
    ax_cl   = fig.add_axes([0.20, 0.02, 0.12, 0.04])
    ax_stop = fig.add_axes([0.34, 0.02, 0.12, 0.04])
    ax_rst  = fig.add_axes([0.48, 0.02, 0.12, 0.04])
    ax_thr  = fig.add_axes([0.66, 0.03, 0.28, 0.02])

    btn_ol   = Button(ax_ol,   'Open-Loop',   color=BTN_GRN, hovercolor='#3d7a3d')
    btn_cl   = Button(ax_cl,   'Closed-Loop', color=BTN_BLU, hovercolor='#3d6a7a')
    btn_stop = Button(ax_stop, 'Stop Motor',  color=BTN_RED, hovercolor='#7a3d3d')
    btn_rst  = Button(ax_rst,  'Reset FC',    color=BTN_RED, hovercolor='#7a3d3d')
    slider_thr = Slider(ax_thr, 'Throttle', 0.0, 0.50, valinit=0.05, color='#55aaff')

    for b in [btn_ol, btn_cl, btn_stop, btn_rst]:
        b.label.set_color(TEXT)
        b.label.set_fontsize(9)

    def start_mode(log_class, mode_name):
        """Stop motor → calibrate → set log class → throttle."""
        mode[0] = mode_name
        # Stop first
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0)))
        ser.flush()
        time.sleep(0.5)
        # Calibrate
        ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([LOG_CLASS_FOC])))
        ser.flush()
        time.sleep(5)  # 4 cal steps × 1s + margin
        # Set log class
        ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([log_class])))
        ser.flush()
        time.sleep(0.1)
        # Start motor
        thr = slider_thr.val
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', thr)))
        ser.flush()

    def on_openloop(evt):
        threading.Thread(target=start_mode, args=(LOG_CLASS_VOLTAGE, 'Open-Loop'), daemon=True).start()

    def on_closedloop(evt):
        threading.Thread(target=start_mode, args=(LOG_CLASS_ENCODER, 'Closed-Loop'), daemon=True).start()

    def on_stop(evt):
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0)))
        ser.flush()
        mode[0] = 'none'

    def on_reset(evt):
        ser.write(build_frame(DB_CMD_RESET, b''))
        ser.flush()
        mode[0] = 'none'

    btn_ol.on_clicked(on_openloop)
    btn_cl.on_clicked(on_closedloop)
    btn_stop.on_clicked(on_stop)
    btn_rst.on_clicked(on_reset)

    def on_throttle(val):
        if last_state[0] >= 2:
            ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', val)))
            ser.flush()

    slider_thr.on_changed(on_throttle)

    # ── Animation ───────────────────────────────────────────────────────
    def update(frame_num):
        nonlocal h_id, h_iq, h_iqref, h_ia, h_ib, h_theta, h_vd, h_vq

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

                if mode[0] == 'Open-Loop':
                    # LOG_CLASS_VOLTAGE (3):
                    # [0]state [1]Id [2]Iq [3]theta [4]enc_elec [5]enc_offset [6]Iα [7]Iβ
                    h_id    = np.roll(h_id,    -1); h_id[-1]    = f[1]
                    h_iq    = np.roll(h_iq,    -1); h_iq[-1]    = f[2]
                    h_theta = np.roll(h_theta, -1); h_theta[-1] = math.degrees(f[3]) % 360
                    h_ia    = np.roll(h_ia,    -1); h_ia[-1]    = f[6]
                    h_ib    = np.roll(h_ib,    -1); h_ib[-1]    = f[7]
                    h_vd    = np.roll(h_vd,    -1); h_vd[-1]    = 0
                    h_vq    = np.roll(h_vq,    -1); h_vq[-1]    = 0
                    h_iqref = np.roll(h_iqref, -1); h_iqref[-1] = 0
                elif mode[0] == 'Closed-Loop':
                    # LOG_CLASS_ENCODER (5):
                    # [0]state [1]Id [2]Iq [3]Vd [4]Vq [5]Iq_ref [6]Id_ref [7]theta
                    h_id    = np.roll(h_id,    -1); h_id[-1]    = f[1]
                    h_iq    = np.roll(h_iq,    -1); h_iq[-1]    = f[2]
                    h_vd    = np.roll(h_vd,    -1); h_vd[-1]    = f[3]
                    h_vq    = np.roll(h_vq,    -1); h_vq[-1]    = f[4]
                    h_iqref = np.roll(h_iqref, -1); h_iqref[-1] = f[5]
                    h_theta = np.roll(h_theta, -1); h_theta[-1] = math.degrees(f[7]) % 360
                    # No Clarke data in this mode
                    h_ia = np.roll(h_ia, -1); h_ia[-1] = 0
                    h_ib = np.roll(h_ib, -1); h_ib[-1] = 0
                else:
                    # Calibration or unknown — just show state
                    pass

                sname = STATE_NAMES.get(state, f'?{state}')
                status_text.set_text(sname)
                status_text.set_color(STATE_COLORS.get(state, '#888'))

        # Update mode text
        if mode[0] != 'none':
            mode_text.set_text(mode[0])
        else:
            mode_text.set_text('')

        info_text.set_text(f'Id={h_id[-1]:+.3f}  Iq={h_iq[-1]:+.3f}  θ={h_theta[-1]:.0f}°')

        # Update plots
        line_id.set_data(x, h_id)
        line_iq.set_data(x, h_iq)
        line_iqref.set_data(x, h_iqref)
        ax_idiq.relim(); ax_idiq.autoscale_view(scalex=False)

        line_ia.set_data(x, h_ia)
        line_ib.set_data(x, h_ib)
        ax_clarke.relim(); ax_clarke.autoscale_view(scalex=False)

        line_theta.set_data(x, h_theta)

        line_vd.set_data(x, h_vd)
        line_vq.set_data(x, h_vq)
        ax_vdvq.relim(); ax_vdvq.autoscale_view(scalex=False)

        n = 100
        line_liss.set_data(h_ia[-n:], h_ib[-n:])
        ax_liss.relim(); ax_liss.autoscale_view()

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
