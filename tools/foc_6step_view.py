#!/usr/bin/env python3
"""
ESC 6-Step View Tool — BEMF Burst Capture

Captures 720 consecutive 40kHz ADC samples (~1 electrical revolution at 55 ERPS)
and plots the full BEMF waveform for all 3 phases plus computed neutral point.

Flow:
  1. Press Start — motor runs at throttle slider value
  2. Wait for motor to stabilize (~2 seconds)
  3. Press Capture — firmware captures 720 samples at 40kHz (18ms), streams back
  4. Waveform appears after ~5 seconds of streaming

Buttons:
  Start     — start motor with current throttle
  Capture   — trigger burst capture (LOG_CLASS 4)
  Stop      — stop motor (throttle=0)
  Reset     — hardware reset

Usage:
  python3 foc_6step_view.py
"""

import serial, serial.tools.list_ports, struct, threading, queue, time, sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider

BAUD = 19200
DB_CMD_THROTTLE  = 0x01
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07
LOG_CLASS_BEMF_BURST = 4
BURST_SAMPLES = 720

# ── Theme ───────────────────────────────────────────────────────────────────
BG      = '#0d1117'
PANEL   = '#161b22'
TEXT    = '#e6edf3'
DIM     = '#7d8590'
GRID    = '#21262d'
ACCENT  = '#58a6ff'
GREEN   = '#3fb950'

BTN_GRN = '#238636'
BTN_RED = '#da3633'
BTN_GRY = '#30363d'
BTN_BLU = '#1f6feb'

# Phase colors
CLR_U = '#ff6b6b'
CLR_V = '#6bff6b'
CLR_W = '#6ba4ff'
CLR_N = '#ffffff'

STEP_COLORS = ['#ff6b6b', '#ffb347', '#6bff6b', '#47d4ff', '#6ba4ff', '#bc8cff']


def find_port():
    for p, d, h in sorted(serial.tools.list_ports.comports()):
        if 'usbmodem31203' in p:
            return p
    for p, d, h in sorted(serial.tools.list_ports.comports()):
        if any(x in p for x in ['usbmodem', 'usbserial', 'ttyACM', 'ttyUSB']):
            return p
    return None


def build_frame(cmd, payload):
    hdr = struct.pack('<2sBBH', b'db', cmd, 0, len(payload))
    ck = cmd + 0 + (len(payload) & 0xFF) + ((len(payload) >> 8) & 0xFF)
    for b in payload:
        ck += b
    return hdr + payload + struct.pack('<H', ck & 0xFFFF)


class FrameParser:
    S1, S2, CMD, CLS, SL, SH, PL, CL, CH = range(9)

    def __init__(self):
        self.state = self.S1
        self.cmd = 0
        self.size = 0
        self.payload = bytearray()
        self.ck_acc = 0
        self.ck_lo = 0

    def feed(self, data):
        for b in data:
            if self.state == self.S1:
                if b == ord('d'):
                    self.state = self.S2
            elif self.state == self.S2:
                self.state = self.CMD if b == ord('b') else self.S1
            elif self.state == self.CMD:
                self.cmd = b
                self.ck_acc = b
                self.state = self.CLS
            elif self.state == self.CLS:
                self.ck_acc += b
                self.state = self.SL
            elif self.state == self.SL:
                self.size = b
                self.ck_acc += b
                self.state = self.SH
            elif self.state == self.SH:
                self.size |= b << 8
                self.ck_acc += b
                if self.size > 120:
                    self.state = self.S1
                elif self.size == 0:
                    self.state = self.CL
                else:
                    self.payload = bytearray()
                    self.state = self.PL
            elif self.state == self.PL:
                self.payload.append(b)
                self.ck_acc += b
                if len(self.payload) >= self.size:
                    self.state = self.CL
            elif self.state == self.CL:
                self.ck_lo = b
                self.state = self.CH
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
    if not port:
        print("No serial port found")
        sys.exit(1)
    print(f"Port: {port}")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    q = queue.Queue(maxsize=500)
    stop_evt = threading.Event()
    threading.Thread(target=serial_reader, args=(ser, q, stop_evt), daemon=True).start()

    plt.rcParams.update({
        'figure.facecolor': BG,
        'axes.facecolor': PANEL,
        'axes.edgecolor': GRID,
        'axes.labelcolor': DIM,
        'xtick.color': DIM,
        'ytick.color': DIM,
        'text.color': TEXT,
        'grid.color': GRID,
        'grid.alpha': 0.3,
        'font.size': 9,
    })

    fig, (ax_bemf, ax_step) = plt.subplots(2, 1, figsize=(14, 7),
        gridspec_kw={'height_ratios': [4, 1]}, sharex=True)
    fig.canvas.manager.set_window_title('ESC 6-Step — BEMF Burst Capture')
    fig.subplots_adjust(left=0.06, right=0.96, top=0.93, bottom=0.16, hspace=0.08)

    # BEMF waveform plot
    ax_bemf.set_title('BEMF ADC — Burst Capture (40 kHz)', fontsize=11, pad=6)
    ax_bemf.set_ylabel('ADC (10-bit)', fontsize=9)
    ax_bemf.set_xlim(0, BURST_SAMPLES)
    ax_bemf.set_ylim(0, 1023)
    ax_bemf.grid(True, linewidth=0.3, alpha=0.4)

    line_u, = ax_bemf.plot([], [], color=CLR_U, lw=0.8, label='U')
    line_v, = ax_bemf.plot([], [], color=CLR_V, lw=0.8, label='V')
    line_w, = ax_bemf.plot([], [], color=CLR_W, lw=0.8, label='W')
    line_n, = ax_bemf.plot([], [], color=CLR_N, lw=0.6, ls='--', alpha=0.5, label='Neutral')
    ax_bemf.legend(fontsize=8, loc='upper right')

    # Step plot
    ax_step.set_ylabel('Step', fontsize=9)
    ax_step.set_xlabel('Sample (@ 40 kHz = 25 µs/sample)', fontsize=8)
    ax_step.set_ylim(-0.5, 5.5)
    ax_step.set_yticks(range(6))
    ax_step.grid(True, linewidth=0.3, alpha=0.4)
    line_step, = ax_step.plot([], [], color=GREEN, lw=1, drawstyle='steps-post')

    status_text = fig.text(0.50, 0.96, 'Ready — press Start then Capture',
                           fontsize=10, ha='center', color=DIM)

    # ── Burst buffers ───────────────────────────────────────────────────
    burst_u = np.zeros(BURST_SAMPLES)
    burst_v = np.zeros(BURST_SAMPLES)
    burst_w = np.zeros(BURST_SAMPLES)
    burst_step_arr = np.zeros(BURST_SAMPLES)
    burst_zc = np.zeros(BURST_SAMPLES)
    burst_received = [0]
    motor_running = [False]
    step_spans = []

    # ── Buttons ─────────────────────────────────────────────────────────
    ax_b1 = fig.add_axes([0.06, 0.03, 0.10, 0.05])
    ax_b2 = fig.add_axes([0.17, 0.03, 0.10, 0.05])
    ax_b3 = fig.add_axes([0.28, 0.03, 0.08, 0.05])
    ax_b4 = fig.add_axes([0.37, 0.03, 0.08, 0.05])
    ax_thr = fig.add_axes([0.50, 0.04, 0.44, 0.025])

    btn_start   = Button(ax_b1, 'Start',   color=BTN_GRN, hovercolor='#2ea043')
    btn_capture = Button(ax_b2, 'Capture', color=BTN_BLU, hovercolor='#388bfd')
    btn_stop    = Button(ax_b3, 'Stop',    color=BTN_RED, hovercolor='#f85149')
    btn_rst     = Button(ax_b4, 'Reset',   color=BTN_GRY, hovercolor='#484f58')
    slider_thr  = Slider(ax_thr, 'Throttle', 0.0, 1.0, valinit=0.10, color=ACCENT)

    for b in [btn_start, btn_capture, btn_stop, btn_rst]:
        b.label.set_color(TEXT)
        b.label.set_fontsize(9)

    def start_motor(_):
        thr = slider_thr.val
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', thr)))
        ser.flush()
        motor_running[0] = True
        status_text.set_text(f'Motor started at {thr:.0%} — press Capture for burst')
        status_text.set_color(GREEN)

    def trigger_capture(_):
        burst_received[0] = 0
        status_text.set_text('Capturing 720 samples at 40 kHz...')
        status_text.set_color(ACCENT)
        ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([LOG_CLASS_BEMF_BURST])))
        ser.flush()

    def on_stop(_):
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0)))
        ser.flush()
        motor_running[0] = False
        status_text.set_text('Motor stopped')
        status_text.set_color(DIM)

    def on_reset(_):
        ser.write(build_frame(DB_CMD_RESET, b''))
        ser.flush()
        motor_running[0] = False
        status_text.set_text('Reset sent')
        status_text.set_color(DIM)

    btn_start.on_clicked(start_motor)
    btn_capture.on_clicked(trigger_capture)
    btn_stop.on_clicked(on_stop)
    btn_rst.on_clicked(on_reset)

    def on_throttle(val):
        if motor_running[0]:
            ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', val)))
            ser.flush()
    slider_thr.on_changed(on_throttle)

    # ── Update loop ─────────────────────────────────────────────────────
    def update(_):
        nonlocal step_spans
        count = 0
        got_complete = False

        while not q.empty() and count < 50:
            try:
                cmd, payload = q.get_nowait()
            except queue.Empty:
                break
            count += 1

            if cmd == 0x00 and len(payload) >= 48:
                f = struct.unpack('<12f', payload[:48])

                if f[0] < 0:  # Burst marker (-1.0)
                    offset = int(f[1])
                    total = int(f[2])

                    # Unpack 9 floats as raw bytes -> 18 uint16 = 6 samples x 3
                    raw = struct.pack('<9f', *f[3:12])
                    samples = struct.unpack('<18H', raw)

                    for i in range(6):
                        idx = offset + i
                        if idx >= total:
                            break
                        packed_u = samples[i * 3]
                        burst_u[idx] = packed_u & 0x3FF
                        burst_step_arr[idx] = (packed_u >> 10) & 0x7
                        burst_zc[idx] = (packed_u >> 13) & 0x1
                        burst_v[idx] = samples[i * 3 + 1] & 0x3FF
                        burst_w[idx] = samples[i * 3 + 2] & 0x3FF

                    burst_received[0] = offset + 6
                    pct = min(100, burst_received[0] * 100 // total)
                    status_text.set_text(f'Receiving... {pct}%')

                    if burst_received[0] >= total:
                        got_complete = True

        if got_complete:
            n = BURST_SAMPLES
            x = np.arange(n)
            neutral = (burst_u[:n] + burst_v[:n] + burst_w[:n]) / 3.0

            line_u.set_data(x, burst_u[:n])
            line_v.set_data(x, burst_v[:n])
            line_w.set_data(x, burst_w[:n])
            line_n.set_data(x, neutral)
            ax_bemf.set_xlim(0, n)
            ax_bemf.relim()
            ax_bemf.autoscale_view(scalex=False)

            line_step.set_data(x, burst_step_arr[:n])
            ax_step.set_xlim(0, n)

            # Color-code step regions on BEMF plot
            for sp in step_spans:
                sp.remove()
            step_spans = []
            steps = burst_step_arr[:n]
            i = 0
            while i < n:
                s = int(steps[i])
                j = i
                while j < n and int(steps[j]) == s:
                    j += 1
                sp = ax_bemf.axvspan(i, j, alpha=0.06, color=STEP_COLORS[s % 6])
                step_spans.append(sp)
                i = j

            status_text.set_text(f'Capture complete — {n} samples')
            status_text.set_color(GREEN)

        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=80)
    timer.add_callback(update, 0)
    timer.start()

    def on_close(e):
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
