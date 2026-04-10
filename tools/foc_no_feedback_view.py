#!/usr/bin/env python3
"""
ESC No-Feedback Test Tool

For firmware built with: ./build.sh no_feedback

Tests open-loop voltage drive. Shows Park currents, Clarke currents,
rotor angle, phase currents, Lissajous, and bus voltage.

Buttons:
  Start     — spin motor open-loop (LOG_CLASS_VOLTAGE)
  Currents  — show Clarke + phase currents (LOG_CLASS_CURRENT)
  Stop      — stop motor
  Reset     — hardware reset

Usage:
  python3 foc_no_feedback_view.py
"""

import serial, serial.tools.list_ports, struct, threading, queue, time, math, sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider

BAUD = 19200
DB_CMD_THROTTLE  = 0x01
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07
LOG_CLASS_VOLTAGE = 3
LOG_CLASS_CURRENT = 2
HISTORY = 300

BG = '#1e1e1e'; PANEL = '#252526'; TEXT = '#cccccc'; DIM = '#888888'; GRID = '#3c3c3c'
BTN_GRN = '#2d5a2d'; BTN_RED = '#5a2d2d'; BTN_BLU = '#2d4a5a'
STATE_NAMES = {0:'IDLE', 1:'ALIGN', 2:'RAMP', 3:'OPENLOOP', 4:'CLOSEDLOOP'}
STATE_COLORS = {0:'#888', 1:'#fa0', 2:'#5af', 3:'#5f5', 4:'#f5f'}

def find_port():
    for p, d, h in sorted(serial.tools.list_ports.comports()):
        if 'usbmodem31203' in p: return p
    for p, d, h in sorted(serial.tools.list_ports.comports()):
        if any(x in p for x in ['usbmodem', 'usbserial', 'ttyACM', 'ttyUSB']): return p
    return None

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

def main():
    port = find_port()
    if not port: print("No serial port found"); sys.exit(1)
    print(f"Port: {port}")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.3); ser.reset_input_buffer()

    q = queue.Queue(maxsize=500)
    stop_evt = threading.Event()
    threading.Thread(target=serial_reader, args=(ser, q, stop_evt), daemon=True).start()

    plt.rcParams.update({
        'figure.facecolor': BG, 'axes.facecolor': PANEL,
        'axes.edgecolor': GRID, 'axes.labelcolor': TEXT,
        'xtick.color': DIM, 'ytick.color': DIM, 'text.color': TEXT,
        'grid.color': GRID, 'grid.alpha': 0.5,
    })

    fig = plt.figure(figsize=(14, 9))
    fig.canvas.manager.set_window_title('ESC No-Feedback View')

    gs = fig.add_gridspec(3, 2, hspace=0.35, wspace=0.25,
                          left=0.06, right=0.97, top=0.95, bottom=0.12)

    ax_idiq   = fig.add_subplot(gs[0, 0])
    ax_clarke = fig.add_subplot(gs[0, 1])
    ax_theta  = fig.add_subplot(gs[1, 0])
    ax_phase  = fig.add_subplot(gs[1, 1])
    ax_liss   = fig.add_subplot(gs[2, 0])
    ax_status = fig.add_subplot(gs[2, 1])

    for ax in [ax_idiq, ax_clarke, ax_theta, ax_phase]:
        ax.set_xlim(0, HISTORY); ax.grid(True, linewidth=0.5)

    ax_idiq.set_title('Park (Id, Iq)', fontsize=10); ax_idiq.set_ylabel('A')
    line_id, = ax_idiq.plot([], [], color='#ff6b6b', lw=1, label='Id')
    line_iq, = ax_idiq.plot([], [], color='#6bff6b', lw=1, label='Iq')
    ax_idiq.legend(fontsize=7, loc='upper right')

    ax_clarke.set_title('Clarke (Ia, Ib)', fontsize=10); ax_clarke.set_ylabel('A')
    line_ia, = ax_clarke.plot([], [], color='#ffaa55', lw=1, label='Ia')
    line_ib, = ax_clarke.plot([], [], color='#55aaff', lw=1, label='Ib')
    ax_clarke.legend(fontsize=7, loc='upper right')

    ax_theta.set_title('Rotor Angle', fontsize=10); ax_theta.set_ylabel('deg')
    ax_theta.set_ylim(-10, 370)
    line_theta, = ax_theta.plot([], [], color='#55ff55', lw=1)

    ax_phase.set_title('Phase Currents', fontsize=10); ax_phase.set_ylabel('A')
    line_iu, = ax_phase.plot([], [], color='#ff6b6b', lw=1, label='Iu')
    line_iv, = ax_phase.plot([], [], color='#6bff6b', lw=1, label='Iv')
    line_iw, = ax_phase.plot([], [], color='#6b6bff', lw=1, label='Iw')
    ax_phase.legend(fontsize=7, loc='upper right')

    ax_liss.set_title('Lissajous (Ia vs Ib)', fontsize=10)
    ax_liss.set_xlabel('Ia (A)'); ax_liss.set_ylabel('Ib (A)')
    ax_liss.set_aspect('equal'); ax_liss.grid(True, linewidth=0.5)
    line_liss, = ax_liss.plot([], [], color='#55aaff', lw=0.8, alpha=0.7)

    ax_status.set_xlim(0, 1); ax_status.set_ylim(0, 1); ax_status.axis('off')
    status_text = ax_status.text(0.5, 0.7, 'IDLE', fontsize=16, ha='center',
                                  va='center', fontweight='bold', color='#888')
    mode_text = ax_status.text(0.5, 0.45, '', fontsize=11, ha='center', va='center', color='#5af')
    info_text = ax_status.text(0.5, 0.2, '', fontsize=9, ha='center', va='center', color=DIM)

    h = {k: np.zeros(HISTORY) for k in ['id','iq','ia','ib','theta','iu','iv','iw','vbus']}
    x = np.arange(HISTORY)
    last_state = [0]
    mode = ['none']

    # Buttons
    ax_b1 = fig.add_axes([0.06, 0.02, 0.12, 0.04])
    ax_b2 = fig.add_axes([0.20, 0.02, 0.12, 0.04])
    ax_b3 = fig.add_axes([0.34, 0.02, 0.10, 0.04])
    ax_b4 = fig.add_axes([0.46, 0.02, 0.08, 0.04])
    ax_thr = fig.add_axes([0.58, 0.03, 0.35, 0.02])

    btn_start = Button(ax_b1, 'Start', color=BTN_GRN, hovercolor='#3d7a3d')
    btn_curr  = Button(ax_b2, 'Currents', color=BTN_BLU, hovercolor='#3d6a7a')
    btn_stop  = Button(ax_b3, 'Stop', color=BTN_RED, hovercolor='#7a3d3d')
    btn_rst   = Button(ax_b4, 'Reset', color=BTN_RED, hovercolor='#7a3d3d')
    slider_thr = Slider(ax_thr, 'Throttle', 0.0, 0.50, valinit=0.05, color='#55aaff')

    for b in [btn_start, btn_curr, btn_stop, btn_rst]:
        b.label.set_color(TEXT); b.label.set_fontsize(9)

    def start_mode(log_class, mode_name):
        mode[0] = mode_name
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0))); ser.flush()
        time.sleep(0.3)
        ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([log_class]))); ser.flush()
        time.sleep(0.1)
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', slider_thr.val))); ser.flush()

    def on_start(e):
        threading.Thread(target=start_mode, args=(LOG_CLASS_VOLTAGE, 'Voltage'), daemon=True).start()
    def on_curr(e):
        threading.Thread(target=start_mode, args=(LOG_CLASS_CURRENT, 'Current'), daemon=True).start()
    def on_stop(e):
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0))); ser.flush()
        mode[0] = 'none'
    def on_reset(e):
        ser.write(build_frame(DB_CMD_RESET, b'')); ser.flush()
        mode[0] = 'none'

    btn_start.on_clicked(on_start)
    btn_curr.on_clicked(on_curr)
    btn_stop.on_clicked(on_stop)
    btn_rst.on_clicked(on_reset)
    def on_throttle(val):
        if last_state[0] >= 2:
            ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', val))); ser.flush()
    slider_thr.on_changed(on_throttle)

    def update(_):
        count = 0
        while not q.empty() and count < 50:
            try: cmd, payload = q.get_nowait()
            except queue.Empty: break
            count += 1
            if cmd == 0x00 and len(payload) == 32:
                f = struct.unpack('<8f', payload)
                state = int(f[0])
                last_state[0] = state
                if mode[0] == 'Voltage':
                    # [0]state [1]Id [2]Iq [3]theta [4]enc_elec [5]enc_offset [6]Ia [7]Ib
                    h['id'] = np.roll(h['id'], -1); h['id'][-1] = f[1]
                    h['iq'] = np.roll(h['iq'], -1); h['iq'][-1] = f[2]
                    h['theta'] = np.roll(h['theta'], -1); h['theta'][-1] = math.degrees(f[3]) % 360
                    h['ia'] = np.roll(h['ia'], -1); h['ia'][-1] = f[6]
                    h['ib'] = np.roll(h['ib'], -1); h['ib'][-1] = f[7]
                elif mode[0] == 'Current':
                    # [0]state [1]Ia [2]Ib [3]enc_elec [4]Iu [5]Iv [6]Iw [7]Vbus
                    h['ia'] = np.roll(h['ia'], -1); h['ia'][-1] = f[1]
                    h['ib'] = np.roll(h['ib'], -1); h['ib'][-1] = f[2]
                    h['iu'] = np.roll(h['iu'], -1); h['iu'][-1] = f[4]
                    h['iv'] = np.roll(h['iv'], -1); h['iv'][-1] = f[5]
                    h['iw'] = np.roll(h['iw'], -1); h['iw'][-1] = f[6]
                    h['vbus'] = np.roll(h['vbus'], -1); h['vbus'][-1] = f[7]
                sname = STATE_NAMES.get(state, f'?{state}')
                status_text.set_text(sname)
                status_text.set_color(STATE_COLORS.get(state, '#888'))

        mode_text.set_text(mode[0] if mode[0] != 'none' else '')
        info_text.set_text(f'Id={h["id"][-1]:+.3f}  Iq={h["iq"][-1]:+.3f}  '
                           f'theta={h["theta"][-1]:.0f}°  Vbus={h["vbus"][-1]:.1f}V')

        line_id.set_data(x, h['id']); line_iq.set_data(x, h['iq'])
        ax_idiq.relim(); ax_idiq.autoscale_view(scalex=False)
        line_ia.set_data(x, h['ia']); line_ib.set_data(x, h['ib'])
        ax_clarke.relim(); ax_clarke.autoscale_view(scalex=False)
        line_theta.set_data(x, h['theta'])
        line_iu.set_data(x, h['iu']); line_iv.set_data(x, h['iv']); line_iw.set_data(x, h['iw'])
        ax_phase.relim(); ax_phase.autoscale_view(scalex=False)
        n = 100; line_liss.set_data(h['ia'][-n:], h['ib'][-n:])
        ax_liss.relim(); ax_liss.autoscale_view()
        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=80)
    timer.add_callback(update, 0); timer.start()

    def on_close(e):
        stop_evt.set()
        try: ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0))); ser.flush()
        except: pass
        ser.close()

    fig.canvas.mpl_connect('close_event', on_close)
    plt.show()

if __name__ == '__main__':
    main()
