#!/usr/bin/env python3
"""
ESC 6-Step View Tool — Enhanced Diagnostic UI

For firmware built with: ./build.sh 6step

Features:
  - Animated rotor/stator visualization with phase coloring
  - Real-time comparator state and ZC diagnostic info
  - Speed, step period, and ZC count history plots
  - Full BEMF troubleshooting data

Buttons:
  Start     — LOG_CLASS_COMMUTATION (3): all diagnostic data
  Stop      — stop motor (throttle=0)
  Reset     — hardware reset

Usage:
  python3 foc_6step_view.py
"""

import serial, serial.tools.list_ports, struct, threading, queue, time, math, sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from matplotlib.patches import Wedge, Circle, Arc
import matplotlib.patheffects as pe

BAUD = 19200
DB_CMD_THROTTLE  = 0x01
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07
LOG_CLASS_COMMUTATION = 3
HISTORY = 300
POLE_PAIRS = 22

# ── Theme ───────────────────────────────────────────────────────────────────
BG      = '#0d1117'
PANEL   = '#161b22'
CARD    = '#1c2128'
TEXT    = '#e6edf3'
DIM     = '#7d8590'
GRID    = '#21262d'
ACCENT  = '#58a6ff'
GREEN   = '#3fb950'
RED     = '#f85149'
ORANGE  = '#d29922'
PURPLE  = '#bc8cff'
CYAN    = '#39d2c0'

BTN_GRN = '#238636'
BTN_RED = '#da3633'
BTN_GRY = '#30363d'

STATE_NAMES  = {0: 'IDLE', 1: 'ALIGN', 2: 'RAMP', 3: 'CLOSED-LOOP'}
STATE_COLORS = {0: DIM, 1: ORANGE, 2: ACCENT, 3: GREEN}

# Phase colors
CLR_U = '#ff6b6b'
CLR_V = '#6bff6b'
CLR_W = '#6ba4ff'
PHASE_COLORS = {'U': CLR_U, 'V': CLR_V, 'W': CLR_W}

# Commutation table: step -> (high/push, low/pull, float/sense)
STEP_TABLE = [
    ('U', 'W', 'V'),
    ('V', 'W', 'U'),
    ('V', 'U', 'W'),
    ('W', 'U', 'V'),
    ('W', 'V', 'U'),
    ('U', 'V', 'W'),
]

# Phase angular positions on stator (120 deg apart)
PHASE_ANGLES = {'U': 90, 'V': 210, 'W': 330}

# ZC polarity (must match firmware)
ZC_POLARITY = [1, 0, 1, 0, 1, 0]


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

    fig = plt.figure(figsize=(16, 10))
    fig.canvas.manager.set_window_title('ESC 6-Step Diagnostic')

    # Layout: left column = rotor + diagnostics, right column = plots
    gs = fig.add_gridspec(4, 3, hspace=0.4, wspace=0.3,
                          left=0.04, right=0.98, top=0.95, bottom=0.10)

    # Left: Rotor visualization (top), Diagnostic panel (bottom)
    ax_rotor = fig.add_subplot(gs[0:2, 0])
    ax_diag  = fig.add_subplot(gs[2:4, 0])

    # Right: 4 plots stacked
    ax_step   = fig.add_subplot(gs[0, 1:3])
    ax_speed  = fig.add_subplot(gs[1, 1:3])
    ax_zc     = fig.add_subplot(gs[2, 1:3])
    ax_period = fig.add_subplot(gs[3, 1:3])

    for ax in [ax_step, ax_speed, ax_zc, ax_period]:
        ax.set_xlim(0, HISTORY)
        ax.grid(True, linewidth=0.3, alpha=0.4)
        ax.tick_params(labelsize=7)

    # ═══════════════════════════════════════════════════════════════════════
    # ROTOR / STATOR VISUALIZATION
    # ═══════════════════════════════════════════════════════════════════════
    ax_rotor.set_xlim(-1.55, 1.55)
    ax_rotor.set_ylim(-1.55, 1.55)
    ax_rotor.set_aspect('equal')
    ax_rotor.axis('off')

    # Draw stator teeth (3 coils at 120 deg)
    stator_patches = {}
    for phase, angle_deg in PHASE_ANGLES.items():
        a = math.radians(angle_deg)
        color = PHASE_COLORS[phase]

        # Stator tooth (wedge)
        w = Wedge((0, 0), 1.35, angle_deg - 25, angle_deg + 25,
                  width=0.35, facecolor=GRID, edgecolor=DIM, linewidth=1.2)
        ax_rotor.add_patch(w)
        stator_patches[phase] = w

        # Phase label
        lx, ly = 1.45 * math.cos(a), 1.45 * math.sin(a)
        ax_rotor.text(lx, ly, phase, fontsize=13, fontweight='bold',
                      ha='center', va='center', color=color,
                      path_effects=[pe.withStroke(linewidth=2, foreground=BG)])

    # Stator ring
    ax_rotor.add_patch(Circle((0, 0), 1.0, fill=False, edgecolor=DIM,
                               linewidth=1.5, linestyle='-'))

    # Rotor (inner circle with magnets)
    ax_rotor.add_patch(Circle((0, 0), 0.55, facecolor=CARD, edgecolor=DIM, linewidth=1.2))

    # Rotor magnets (7 pole pairs = 14 magnets)
    n_magnets = POLE_PAIRS * 2
    rotor_magnets = []
    for i in range(n_magnets):
        a0 = i * 360.0 / n_magnets
        a1 = a0 + 360.0 / n_magnets - 2
        color = '#ff4444' if i % 2 == 0 else '#4488ff'
        w = Wedge((0, 0), 0.52, a0 + 1, a1, width=0.15,
                  facecolor=color, edgecolor='none', alpha=0.6)
        ax_rotor.add_patch(w)
        rotor_magnets.append(w)

    # Center dot
    ax_rotor.add_patch(Circle((0, 0), 0.08, facecolor=DIM, edgecolor='none'))

    # Rotor angle indicator
    rotor_arrow, = ax_rotor.plot([0, 0], [0, 0.45], color=TEXT, linewidth=2,
                                  solid_capstyle='round')

    # Phase status indicators near each coil
    phase_status_texts = {}
    for phase, angle_deg in PHASE_ANGLES.items():
        a = math.radians(angle_deg)
        sx, sy = 0.78 * math.cos(a), 0.78 * math.sin(a)
        t = ax_rotor.text(sx, sy, '', fontsize=7, ha='center', va='center',
                          color=TEXT, fontweight='bold',
                          bbox=dict(boxstyle='round,pad=0.15', facecolor=BG,
                                    edgecolor='none', alpha=0.8))
        phase_status_texts[phase] = t

    # Status text inside rotor
    rotor_state_text = ax_rotor.text(0, -0.25, 'IDLE', fontsize=10,
                                     ha='center', va='center', fontweight='bold', color=DIM)
    rotor_rpm_text = ax_rotor.text(0, 0.15, '', fontsize=9,
                                    ha='center', va='center', color=GREEN, fontweight='bold')

    # ═══════════════════════════════════════════════════════════════════════
    # DIAGNOSTIC PANEL
    # ═══════════════════════════════════════════════════════════════════════
    ax_diag.set_xlim(0, 1)
    ax_diag.set_ylim(0, 1)
    ax_diag.axis('off')

    ax_diag.add_patch(plt.Rectangle((0.02, 0.02), 0.96, 0.96, facecolor=CARD,
                                     edgecolor=GRID, linewidth=1, transform=ax_diag.transAxes,
                                     clip_on=False, zorder=0))

    ax_diag.text(0.5, 0.95, 'DIAGNOSTICS', fontsize=10, fontweight='bold',
                 ha='center', va='top', color=ACCENT)

    def diag_row(y, label, val_color=TEXT):
        ax_diag.text(0.08, y, label, fontsize=8, ha='left', va='center', color=DIM)
        return ax_diag.text(0.92, y, '--', fontsize=9, ha='right', va='center',
                           color=val_color, fontfamily='monospace', fontweight='bold')

    d_state     = diag_row(0.85, 'State', ACCENT)
    d_step      = diag_row(0.75, 'Step')
    d_zc_map    = diag_row(0.65, 'ZC Step Map')
    d_zc_window = diag_row(0.55, 'ZC Window')
    d_zc_count  = diag_row(0.45, 'ZC Count', GREEN)
    d_zc_consec = diag_row(0.35, 'ZC Consec (ramp)', CYAN)
    d_zc_miss   = diag_row(0.25, 'ZC Miss', RED)
    d_vbus      = diag_row(0.05, 'Vbus')

    # ═══════════════════════════════════════════════════════════════════════
    # PLOTS
    # ═══════════════════════════════════════════════════════════════════════

    ax_step.set_title('Commutation Step', fontsize=9, pad=3)
    ax_step.set_ylabel('Step', fontsize=8)
    ax_step.set_ylim(-0.5, 5.5)
    ax_step.set_yticks(range(6))
    line_step, = ax_step.plot([], [], color=GREEN, lw=1.2, drawstyle='steps-post')

    ax_speed.set_title('Speed', fontsize=9, pad=3)
    ax_speed.set_ylabel('eRPS', fontsize=8)
    line_speed, = ax_speed.plot([], [], color=PURPLE, lw=1.2, label='eRPS')
    ax_speed_rpm = ax_speed.twinx()
    ax_speed_rpm.set_ylabel('RPM', fontsize=8, color=DIM)
    ax_speed_rpm.tick_params(axis='y', labelsize=7, labelcolor=DIM)
    line_rpm, = ax_speed_rpm.plot([], [], color=ORANGE, lw=1, label='RPM', linestyle='--')
    ax_speed.legend(fontsize=7, loc='upper left')
    ax_speed_rpm.legend(fontsize=7, loc='upper right')

    ax_zc.set_title('Zero-Crossing', fontsize=9, pad=3)
    ax_zc.set_ylabel('Count', fontsize=8)
    line_zc, = ax_zc.plot([], [], color=ACCENT, lw=1.2, label='ZC total')
    line_consec, = ax_zc.plot([], [], color=CYAN, lw=1.2, label='ZC consec', linestyle='--')
    line_miss, = ax_zc.plot([], [], color=RED, lw=1, label='ZC miss', linestyle=':')
    ax_zc.legend(fontsize=7, loc='upper left', ncol=3)

    ax_period.set_title('Step Period', fontsize=9, pad=3)
    ax_period.set_ylabel('Ticks', fontsize=8)
    line_period, = ax_period.plot([], [], color=ORANGE, lw=1.2, label='Step period')
    line_ramp_p, = ax_period.plot([], [], color=RED, lw=1, label='Ramp period', linestyle=':')
    ax_period.legend(fontsize=7, loc='upper right')

    # ═══════════════════════════════════════════════════════════════════════
    # HISTORY BUFFERS
    # ═══════════════════════════════════════════════════════════════════════
    h = {k: np.zeros(HISTORY) for k in [
        'step', 'speed', 'rpm', 'zc_count', 'step_period', 'ramp_period',
        'duty', 'vbus', 'zc_consec', 'zc_miss', 'step_zc_map', 'zc_window_sum']}
    xdata = np.arange(HISTORY)
    last_state = [0]
    rotor_angle = [0.0]

    # ═══════════════════════════════════════════════════════════════════════
    # BUTTONS
    # ═══════════════════════════════════════════════════════════════════════
    ax_b1  = fig.add_axes([0.04, 0.02, 0.10, 0.04])
    ax_b3  = fig.add_axes([0.15, 0.02, 0.08, 0.04])
    ax_b4  = fig.add_axes([0.24, 0.02, 0.08, 0.04])
    ax_thr = fig.add_axes([0.38, 0.03, 0.56, 0.02])

    btn_start = Button(ax_b1, 'Start', color=BTN_GRN, hovercolor='#2ea043')
    btn_stop  = Button(ax_b3, 'Stop',  color=BTN_RED, hovercolor='#f85149')
    btn_rst   = Button(ax_b4, 'Reset', color=BTN_GRY, hovercolor='#484f58')
    slider_thr = Slider(ax_thr, 'Throttle', 0.0, 1.0, valinit=0.05, color=ACCENT)

    for b in [btn_start, btn_stop, btn_rst]:
        b.label.set_color(TEXT)
        b.label.set_fontsize(9)

    def start_mode(_):
        for k in h:
            h[k][:] = 0
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0)))
        ser.flush()
        time.sleep(0.3)
        ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([LOG_CLASS_COMMUTATION])))
        ser.flush()
        time.sleep(0.1)
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', slider_thr.val)))
        ser.flush()

    def on_stop(_):
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0)))
        ser.flush()

    def on_reset(_):
        ser.write(build_frame(DB_CMD_RESET, b''))
        ser.flush()

    btn_start.on_clicked(
        lambda e: threading.Thread(target=start_mode, args=(e,), daemon=True).start())
    btn_stop.on_clicked(on_stop)
    btn_rst.on_clicked(on_reset)

    def on_throttle(val):
        if last_state[0] >= 1:
            ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', val)))
            ser.flush()
    slider_thr.on_changed(on_throttle)

    # ═══════════════════════════════════════════════════════════════════════
    # UPDATE LOOP
    # ═══════════════════════════════════════════════════════════════════════
    def update(_):
        count = 0
        while not q.empty() and count < 50:
            try:
                cmd, payload = q.get_nowait()
            except queue.Empty:
                break
            count += 1

            if cmd == 0x00 and len(payload) >= 48:
                # 12 floats = 48 bytes (new format)
                f = struct.unpack('<12f', payload[:48])
                state = int(f[0])
                last_state[0] = state
                h['step']        = np.roll(h['step'], -1);        h['step'][-1] = f[1]
                h['speed']       = np.roll(h['speed'], -1);       h['speed'][-1] = f[2]
                h['rpm']         = np.roll(h['rpm'], -1);         h['rpm'][-1] = f[2] * 60.0 / POLE_PAIRS
                h['duty']        = np.roll(h['duty'], -1);        h['duty'][-1] = f[3]
                h['vbus']        = np.roll(h['vbus'], -1);        h['vbus'][-1] = f[4]
                h['step_period'] = np.roll(h['step_period'], -1); h['step_period'][-1] = f[5]
                h['zc_count']    = np.roll(h['zc_count'], -1);    h['zc_count'][-1] = f[6]
                h['ramp_period'] = np.roll(h['ramp_period'], -1); h['ramp_period'][-1] = f[7]
                h['zc_consec']   = np.roll(h['zc_consec'], -1);   h['zc_consec'][-1] = f[8]
                h['zc_miss']     = np.roll(h['zc_miss'], -1);     h['zc_miss'][-1] = f[9]
                # f[10] = comp_raw: c1|(c2<<1)|(c4<<2)
                # f[11] = step_zc_map (bits 0-5) | (zc_window_sum << 8)
                packed = int(f[11])
                h['step_zc_map'] = np.roll(h['step_zc_map'], -1);  h['step_zc_map'][-1] = packed & 0x3F
                h['zc_window_sum'] = np.roll(h['zc_window_sum'], -1); h['zc_window_sum'][-1] = (packed >> 8) & 0xFF

            elif cmd == 0x00 and len(payload) == 32:
                # 8 floats = 32 bytes (legacy format)
                f = struct.unpack('<8f', payload)
                state = int(f[0])
                last_state[0] = state
                h['step']        = np.roll(h['step'], -1);        h['step'][-1] = f[1]
                h['speed']       = np.roll(h['speed'], -1);       h['speed'][-1] = f[2]
                h['rpm']         = np.roll(h['rpm'], -1);         h['rpm'][-1] = f[2] * 60.0 / POLE_PAIRS
                h['duty']        = np.roll(h['duty'], -1);        h['duty'][-1] = f[3]
                h['vbus']        = np.roll(h['vbus'], -1);        h['vbus'][-1] = f[4]
                h['step_period'] = np.roll(h['step_period'], -1); h['step_period'][-1] = f[5]
                h['zc_count']    = np.roll(h['zc_count'], -1);    h['zc_count'][-1] = f[6]
                h['ramp_period'] = np.roll(h['ramp_period'], -1); h['ramp_period'][-1] = f[7]

        state = last_state[0]
        step_i = int(h['step'][-1]) % 6
        rpm_val = h['rpm'][-1]
        zc_map = int(h['step_zc_map'][-1]) & 0x3F
        comp_exp = ZC_POLARITY[step_i]

        # ── Animate rotor ──
        if state >= 2:
            target_elec = step_i * 60.0
            rotor_angle[0] = target_elec / POLE_PAIRS
        ra = math.radians(rotor_angle[0] + 90)
        rotor_arrow.set_data([0, 0.4 * math.cos(ra)], [0, 0.4 * math.sin(ra)])

        # Rotate magnets
        for i, w in enumerate(rotor_magnets):
            base_a = i * 360.0 / n_magnets
            w.set_theta1(base_a + rotor_angle[0] + 1)
            w.set_theta2(base_a + rotor_angle[0] + 360.0 / n_magnets - 2)

        # ── Update stator coil colors ──
        push_, pull_, sense_ = STEP_TABLE[step_i]
        for phase in ['U', 'V', 'W']:
            w = stator_patches[phase]
            st = phase_status_texts[phase]
            if state < 2:
                w.set_facecolor(GRID)
                st.set_text('')
            elif phase == push_:
                w.set_facecolor(PHASE_COLORS[phase])
                w.set_alpha(0.8)
                st.set_text('PWM')
                st.set_color('#000000')
            elif phase == pull_:
                w.set_facecolor('#333333')
                w.set_alpha(0.9)
                st.set_text('GND')
                st.set_color('#ff8888')
            else:
                w.set_facecolor(CARD)
                w.set_alpha(0.5)
                # Show ZC detection status from bitmask for the floating phase's step
                step_ok = (zc_map >> step_i) & 1
                arrow = '^' if comp_exp else 'v'
                match_clr = GREEN if step_ok else RED
                st.set_text(f'BEMF {arrow}')
                st.set_color(match_clr)

        # ── Rotor center text ──
        sname = STATE_NAMES.get(state, f'?{state}')
        rotor_state_text.set_text(sname)
        rotor_state_text.set_color(STATE_COLORS.get(state, DIM))
        rotor_rpm_text.set_text(f'{rpm_val:.0f} RPM' if rpm_val > 0 else '')

        # ── Update diagnostic panel ──
        d_state.set_text(sname)
        d_state.set_color(STATE_COLORS.get(state, DIM))

        d_step.set_text(f'{step_i}  ({push_}+ {pull_}- {sense_}~)')

        # Show ZC step map as binary: which steps detect ZC
        d_zc_map.set_text(f'{zc_map:06b}  (0x{zc_map:02X})')
        d_zc_map.set_color(GREEN if zc_map == 0x3F else ORANGE)

        window_sum = int(h['zc_window_sum'][-1])
        d_zc_window.set_text(f'{window_sum} / 12')
        d_zc_window.set_color(GREEN if window_sum >= 10 else ORANGE)

        d_zc_count.set_text(f'{int(h["zc_count"][-1])}')

        consec = int(h['zc_consec'][-1])
        d_zc_consec.set_text(f'{consec} / 12')
        d_zc_consec.set_color(GREEN if consec >= 12 else CYAN)

        miss = int(h['zc_miss'][-1])
        d_zc_miss.set_text(f'{miss}')
        d_zc_miss.set_color(RED if miss > 0 else DIM)

        d_vbus.set_text(f'{h["vbus"][-1]:.1f} V')

        # ── Update plots ──
        line_step.set_data(xdata, h['step'])

        line_speed.set_data(xdata, h['speed'])
        line_rpm.set_data(xdata, h['rpm'])
        ax_speed.relim()
        ax_speed.autoscale_view(scalex=False)
        ax_speed_rpm.relim()
        ax_speed_rpm.autoscale_view(scalex=False)

        line_zc.set_data(xdata, h['zc_count'])
        line_consec.set_data(xdata, h['zc_consec'])
        line_miss.set_data(xdata, h['zc_miss'])
        ax_zc.relim()
        ax_zc.autoscale_view(scalex=False)

        line_period.set_data(xdata, h['step_period'])
        line_ramp_p.set_data(xdata, h['ramp_period'])
        ax_period.relim()
        ax_period.autoscale_view(scalex=False)

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
