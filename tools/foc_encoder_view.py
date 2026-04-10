#!/usr/bin/env python3
"""
ESC Encoder Test Tool

For firmware built with: ./build.sh encoder

Tests closed-loop encoder drive. Shows Id/Iq, Vq ramp, rotor theta,
encoder offset/direction, and diagnostics.

Buttons:
  Start     — spin motor CL encoder (LOG_CLASS_ENCODER)
  FOC Diag  — show encoder diagnostics (LOG_CLASS_FOC)
  Stop      — stop motor
  Reset     — hardware reset

Usage:
  python3 foc_encoder_view.py
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
LOG_CLASS_ENCODER = 5
LOG_CLASS_FOC     = 1
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
    fig.canvas.manager.set_window_title('ESC Encoder View')

    gs = fig.add_gridspec(3, 2, hspace=0.35, wspace=0.25,
                          left=0.06, right=0.97, top=0.95, bottom=0.12)

    ax_idiq  = fig.add_subplot(gs[0, 0])
    ax_vq    = fig.add_subplot(gs[0, 1])
    ax_theta = fig.add_subplot(gs[1, 0])
    ax_enc   = fig.add_subplot(gs[1, 1])
    ax_diag  = fig.add_subplot(gs[2, 0])
    ax_status = fig.add_subplot(gs[2, 1])

    for ax in [ax_idiq, ax_vq, ax_theta, ax_enc, ax_diag]:
        ax.set_xlim(0, HISTORY); ax.grid(True, linewidth=0.5)

    ax_idiq.set_title('Park (Id, Iq)', fontsize=10); ax_idiq.set_ylabel('A')
    line_id, = ax_idiq.plot([], [], color='#ff6b6b', lw=1, label='Id')
    line_iq, = ax_idiq.plot([], [], color='#6bff6b', lw=1, label='Iq')
    ax_idiq.legend(fontsize=7, loc='upper right')

    ax_vq.set_title('Voltage Vq', fontsize=10); ax_vq.set_ylabel('V')
    line_vq, = ax_vq.plot([], [], color='#ff55ff', lw=1, label='Vq')
    line_cl_vq, = ax_vq.plot([], [], color='#55ffff', lw=1, label='cl_vq')
    ax_vq.legend(fontsize=7, loc='upper right')

    ax_theta.set_title('Rotor Angle (theta)', fontsize=10); ax_theta.set_ylabel('deg')
    ax_theta.set_ylim(-10, 370)
    line_theta, = ax_theta.plot([], [], color='#55ff55', lw=1)

    ax_enc.set_title('Encoder (elec / mech)', fontsize=10); ax_enc.set_ylabel('rad / deg')
    line_enc_elec, = ax_enc.plot([], [], color='#ffaa55', lw=1, label='enc_elec (rad)')
    line_enc_mech, = ax_enc.plot([], [], color='#55aaff', lw=1, label='enc_mech (deg)')
    ax_enc.legend(fontsize=7, loc='upper right')

    ax_diag.set_title('Enc Offset / Dir', fontsize=10); ax_diag.set_ylabel('rad / flag')
    line_offset, = ax_diag.plot([], [], color='#ffaa55', lw=1, label='offset')
    line_dir, = ax_diag.plot([], [], color='#55ffff', lw=1, label='dir')
    ax_diag.legend(fontsize=7, loc='upper right')

    ax_status.set_xlim(0, 1); ax_status.set_ylim(0, 1); ax_status.axis('off')
    status_text = ax_status.text(0.5, 0.7, 'IDLE', fontsize=16, ha='center',
                                  va='center', fontweight='bold', color='#888')
    mode_text = ax_status.text(0.5, 0.45, '', fontsize=11, ha='center', va='center', color='#5af')
    info_text = ax_status.text(0.5, 0.2, '', fontsize=9, ha='center', va='center', color=DIM)

    h = {k: np.zeros(HISTORY) for k in
         ['id','iq','vq','cl_vq','theta','enc_elec','enc_mech','offset','dir','vbus',
          'duty_a','duty_b','duty_c','v_alpha','v_beta']}
    x = np.arange(HISTORY)
    last_state = [0]
    mode = ['none']

    # Buttons
    ax_b1 = fig.add_axes([0.06, 0.02, 0.10, 0.04])
    ax_b2 = fig.add_axes([0.17, 0.02, 0.10, 0.04])
    ax_b3 = fig.add_axes([0.28, 0.02, 0.10, 0.04])
    ax_b4 = fig.add_axes([0.39, 0.02, 0.08, 0.04])
    ax_b5 = fig.add_axes([0.48, 0.02, 0.08, 0.04])
    ax_thr = fig.add_axes([0.58, 0.03, 0.35, 0.02])

    btn_start = Button(ax_b1, 'Start', color=BTN_GRN, hovercolor='#3d7a3d')
    btn_diag  = Button(ax_b2, 'FOC Diag', color=BTN_BLU, hovercolor='#3d6a7a')
    btn_duty  = Button(ax_b3, 'Duty', color=BTN_BLU, hovercolor='#3d6a7a')
    btn_stop  = Button(ax_b4, 'Stop', color=BTN_RED, hovercolor='#7a3d3d')
    btn_rst   = Button(ax_b5, 'Reset', color=BTN_RED, hovercolor='#7a3d3d')
    slider_thr = Slider(ax_thr, 'Throttle', 0.0, 0.50, valinit=0.05, color='#55aaff')

    for b in [btn_start, btn_diag, btn_duty, btn_stop, btn_rst]:
        b.label.set_color(TEXT); b.label.set_fontsize(9)

    def start_mode(log_class, mode_name):
        mode[0] = mode_name
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0))); ser.flush()
        time.sleep(0.3)
        ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([log_class]))); ser.flush()
        time.sleep(0.1)
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', slider_thr.val))); ser.flush()

    def on_start(e):
        threading.Thread(target=start_mode, args=(LOG_CLASS_ENCODER, 'Encoder'), daemon=True).start()
    def on_diag(e):
        threading.Thread(target=start_mode, args=(LOG_CLASS_FOC, 'FOC'), daemon=True).start()
    def on_duty(e):
        threading.Thread(target=start_mode, args=(LOG_CLASS_CURRENT, 'Duty'), daemon=True).start()
    def on_stop(e):
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0))); ser.flush()
        mode[0] = 'none'
    def on_reset(e):
        ser.write(build_frame(DB_CMD_RESET, b'')); ser.flush()
        mode[0] = 'none'

    btn_start.on_clicked(on_start)
    btn_diag.on_clicked(on_diag)
    btn_duty.on_clicked(on_duty)
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
                if mode[0] == 'Encoder':
                    # [0]state [1]Id [2]Iq [3]enc_elec [4]Vq [5]cl_vq [6]enc_dir [7]theta
                    h['id'] = np.roll(h['id'], -1); h['id'][-1] = f[1]
                    h['iq'] = np.roll(h['iq'], -1); h['iq'][-1] = f[2]
                    h['enc_elec'] = np.roll(h['enc_elec'], -1); h['enc_elec'][-1] = f[3]
                    h['vq'] = np.roll(h['vq'], -1); h['vq'][-1] = f[4]
                    h['cl_vq'] = np.roll(h['cl_vq'], -1); h['cl_vq'][-1] = f[5]
                    h['dir'] = np.roll(h['dir'], -1); h['dir'][-1] = f[6]
                    h['theta'] = np.roll(h['theta'], -1); h['theta'][-1] = math.degrees(f[7]) % 360
                elif mode[0] == 'FOC':
                    # [0]state [1]enc_elec [2]enc_mech [3]theta [4]enc_offset [5]Vbus [6]enc_dir [7]0
                    h['enc_elec'] = np.roll(h['enc_elec'], -1); h['enc_elec'][-1] = f[1]
                    h['enc_mech'] = np.roll(h['enc_mech'], -1); h['enc_mech'][-1] = f[2]
                    h['theta'] = np.roll(h['theta'], -1); h['theta'][-1] = math.degrees(f[3]) % 360
                    h['offset'] = np.roll(h['offset'], -1); h['offset'][-1] = f[4]
                    h['vbus'] = np.roll(h['vbus'], -1); h['vbus'][-1] = f[5]
                    h['dir'] = np.roll(h['dir'], -1); h['dir'][-1] = f[6]
                elif mode[0] == 'Duty':
                    # [0]state [1]duty_a [2]duty_b [3]duty_c [4]v_alpha [5]v_beta [6]Vbus [7]cl_vq
                    h['duty_a'] = np.roll(h['duty_a'], -1); h['duty_a'][-1] = f[1]
                    h['duty_b'] = np.roll(h['duty_b'], -1); h['duty_b'][-1] = f[2]
                    h['duty_c'] = np.roll(h['duty_c'], -1); h['duty_c'][-1] = f[3]
                    h['v_alpha'] = np.roll(h['v_alpha'], -1); h['v_alpha'][-1] = f[4]
                    h['v_beta'] = np.roll(h['v_beta'], -1); h['v_beta'][-1] = f[5]
                    h['vbus'] = np.roll(h['vbus'], -1); h['vbus'][-1] = f[6]
                    h['cl_vq'] = np.roll(h['cl_vq'], -1); h['cl_vq'][-1] = f[7]
                sname = STATE_NAMES.get(state, f'?{state}')
                status_text.set_text(sname)
                status_text.set_color(STATE_COLORS.get(state, '#888'))

        mode_text.set_text(mode[0] if mode[0] != 'none' else '')
        if mode[0] == 'Duty':
            info_text.set_text(f'Da={h["duty_a"][-1]:.0f}  Db={h["duty_b"][-1]:.0f}  '
                               f'Dc={h["duty_c"][-1]:.0f}  Vbus={h["vbus"][-1]:.1f}V  '
                               f'Vq={h["cl_vq"][-1]:+.2f}')
        else:
            info_text.set_text(f'Id={h["id"][-1]:+.3f}  Iq={h["iq"][-1]:+.3f}  '
                               f'Vq={h["vq"][-1]:+.2f}  theta={h["theta"][-1]:.0f}\u00b0  '
                               f'dir={h["dir"][-1]:+.0f}  enc_e={h["enc_elec"][-1]:.2f}')

        line_id.set_data(x, h['id']); line_iq.set_data(x, h['iq'])
        ax_idiq.relim(); ax_idiq.autoscale_view(scalex=False)
        if mode[0] == 'Duty':
            line_vq.set_data(x, h['v_alpha']); line_cl_vq.set_data(x, h['v_beta'])
            ax_vq.set_title('v_alpha / v_beta', fontsize=10)
        else:
            line_vq.set_data(x, h['vq']); line_cl_vq.set_data(x, h['cl_vq'])
            ax_vq.set_title('Voltage Vq', fontsize=10)
        ax_vq.relim(); ax_vq.autoscale_view(scalex=False)
        line_theta.set_data(x, h['theta'])
        if mode[0] == 'Duty':
            line_enc_elec.set_data(x, h['duty_a']); line_enc_mech.set_data(x, h['duty_b'])
            ax_enc.set_title('Duty A / B', fontsize=10)
        else:
            line_enc_elec.set_data(x, h['enc_elec']); line_enc_mech.set_data(x, h['enc_mech'])
            ax_enc.set_title('Encoder elec (rad)', fontsize=10)
        ax_enc.relim(); ax_enc.autoscale_view(scalex=False)
        if mode[0] == 'Duty':
            line_offset.set_data(x, h['duty_c']); line_dir.set_data(x, h['cl_vq'])
            ax_diag.set_title('Duty C / cl_vq', fontsize=10)
        else:
            line_offset.set_data(x, h['offset']); line_dir.set_data(x, h['dir'])
            ax_diag.set_title('Enc Offset / Dir', fontsize=10)
        ax_diag.relim(); ax_diag.autoscale_view(scalex=False)
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
