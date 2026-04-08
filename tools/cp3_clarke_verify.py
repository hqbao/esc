import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
import time
import math

"""
CP3: Clarke Transform Verification

Spins motor in open-loop, logs Iα and Iβ (Clarke outputs).
PASS criteria:
  - Iα and Iβ are sinusoidal
  - ~90° phase shift between them
  - Lissajous plot (Iα vs Iβ) forms a circle/ellipse

Log format (LOG_CLASS_CURRENT = 2, 8 floats):
  [0] state          (0=IDLE, 1=CAL, 2=ALIGN, 3=RAMP, 4=OPENLOOP, 5=CL)
  [1] Iα             (Amps, Clarke alpha)
  [2] Iβ             (Amps, Clarke beta)
  [3] enc_elec       (rad, encoder electrical angle)
  [4] Vbus           (Volts)
  [5..7] reserved    (0)

Usage:
  python3 cp3_clarke_verify.py
  1. Slide throttle to ~0.05–0.10
  2. Click "Start Log" — motor spins, Clarke data streams
  3. Look for clean sinusoids and a circular Lissajous
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 19200
SEND_LOG_ID = 0x00

DB_CMD_THROTTLE  = 0x01
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07
LOG_CLASS_CURRENT = 2

# --- Dark Theme ---
BG_COLOR     = '#1e1e1e'
TEXT_COLOR    = '#cccccc'
DIM_TEXT      = '#888888'
GRID_COLOR   = '#3c3c3c'
BTN_GREEN    = '#2d5a2d'
BTN_RED      = '#5a2d2d'

STATE_NAMES = {
    0: 'IDLE', 1: 'CALIBRATE', 2: 'ALIGN',
    3: 'RAMP', 4: 'OPENLOOP', 5: 'CLOSEDLOOP'
}

HISTORY_LEN = 300

# --- Auto-detect serial port (prefer STLink VCP: VID 0483) ---
ports = serial.tools.list_ports.comports()
print("Scanning for serial ports...")
for port, desc, hwid in sorted(ports):
    if '0483' in hwid:
        SERIAL_PORT = port
        print(f"  \u2713 Auto-selected: {port} ({desc}) [STLink]")
        break
    elif any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB']):
        if not SERIAL_PORT:
            SERIAL_PORT = port
        print(f"  \u00b7 Candidate: {port} ({desc})")
    else:
        print(f"  \u00b7 Skipped: {port} ({desc})")

if not SERIAL_PORT:
    print("  \u2717 No compatible serial port found.")
else:
    print(f"  \u2713 Connected to {SERIAL_PORT}")

# --- Global State ---
data_queue = queue.Queue()
g_serial = None
g_throttle = 0.0


def build_db_frame(cmd_id, payload):
    msg_class = 0x00
    length = len(payload)
    header = struct.pack('<2sBBH', b'db', cmd_id, msg_class, length)
    ck = cmd_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
    for b in payload:
        ck += b
    ck &= 0xFFFF
    return header + payload + struct.pack('<H', ck)


def send_throttle(ser, throttle):
    payload = struct.pack('<f', throttle)
    frame = build_db_frame(DB_CMD_THROTTLE, payload)
    ser.write(frame)
    ser.flush()


def send_log_class(ser, log_class):
    payload = bytes([log_class])
    frame = build_db_frame(DB_CMD_LOG_CLASS, payload)
    ser.write(frame)
    ser.flush()
    print(f"  \u2192 Log class: {log_class}")


def send_reset(ser):
    payload = bytes([0x00])
    frame = build_db_frame(DB_CMD_RESET, payload)
    ser.write(frame)
    ser.flush()
    print("  \u2192 Reset command sent")


def serial_reader():
    global g_serial
    if not SERIAL_PORT:
        return
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.2)
        ser.reset_input_buffer()
        g_serial = ser
        print(f"  \u2713 Connected to {SERIAL_PORT}")

        while True:
            b1 = ser.read(1)
            if not b1:
                continue
            if b1[0] != ord('d'):
                continue
            b2 = ser.read(1)
            if not b2 or b2[0] != ord('b'):
                continue
            hdr = ser.read(4)
            if len(hdr) < 4:
                continue
            msg_id = hdr[0]
            length = hdr[2] | (hdr[3] << 8)
            if length > 120:
                continue
            payload = ser.read(length)
            if len(payload) != length:
                continue
            ck_bytes = ser.read(2)
            if len(ck_bytes) < 2:
                continue

            # Validate checksum
            ck_expect = ck_bytes[0] | (ck_bytes[1] << 8)
            ck_actual = hdr[0] + hdr[1] + hdr[2] + hdr[3]
            for b in payload:
                ck_actual += b
            ck_actual &= 0xFFFF
            if ck_actual != ck_expect:
                continue

            if msg_id == SEND_LOG_ID and length == 32:
                vals = struct.unpack('<8f', payload)
                data_queue.put(vals)
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


def main():
    global g_throttle

    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    plt.style.use('dark_background')
    plt.rcParams.update({
        'figure.facecolor': BG_COLOR,
        'axes.facecolor': BG_COLOR,
        'axes.edgecolor': GRID_COLOR,
        'axes.labelcolor': TEXT_COLOR,
        'xtick.color': DIM_TEXT,
        'ytick.color': DIM_TEXT,
        'text.color': TEXT_COLOR,
        'grid.color': GRID_COLOR,
    })

    fig = plt.figure(figsize=(14, 10))
    fig.canvas.manager.set_window_title('CP3: Clarke Transform Verification')

    gs = fig.add_gridspec(3, 3, height_ratios=[1, 1, 0.35],
                          hspace=0.4, wspace=0.3,
                          left=0.08, right=0.95, top=0.95, bottom=0.06)

    ax_clarke = fig.add_subplot(gs[0, :])    # Iα, Iβ vs time
    ax_phase  = fig.add_subplot(gs[1, 0])    # Iu, Iv, Iw vs time
    ax_liss   = fig.add_subplot(gs[1, 1])    # Lissajous: Iα vs Iβ
    ax_info   = fig.add_subplot(gs[1, 2])    # Status text

    ax_clarke.set_title('Clarke Currents (I\u03b1, I\u03b2) vs Time', fontsize=10)
    ax_clarke.set_ylabel('Current (A)')
    ax_clarke.grid(True, alpha=0.3)

    ax_phase.set_title('Phase Currents (Iu, Iv, Iw) vs Time', fontsize=10)
    ax_phase.set_ylabel('Current (A)')
    ax_phase.grid(True, alpha=0.3)

    ax_liss.set_title('Lissajous: I\u03b1 vs I\u03b2', fontsize=10)
    ax_liss.set_xlabel('I\u03b1 (A)')
    ax_liss.set_ylabel('I\u03b2 (A)')
    ax_liss.set_aspect('equal')
    ax_liss.grid(True, alpha=0.3)

    ax_info.axis('off')
    ax_info.set_title('Status', fontsize=10)
    info_text = ax_info.text(0.05, 0.95, 'Waiting for data...',
                              fontsize=11, color=TEXT_COLOR,
                              va='top', family='monospace',
                              transform=ax_info.transAxes)

    # History arrays
    h_ia = np.zeros(HISTORY_LEN)
    h_ib = np.zeros(HISTORY_LEN)
    h_iu = np.zeros(HISTORY_LEN)
    h_iv = np.zeros(HISTORY_LEN)
    h_iw = np.zeros(HISTORY_LEN)

    line_ia, = ax_clarke.plot([], [], color='#55ff55', lw=1.2, label='I\u03b1')
    line_ib, = ax_clarke.plot([], [], color='#ff5555', lw=1.2, label='I\u03b2')
    ax_clarke.legend(loc='upper right', fontsize=8)

    line_iu, = ax_phase.plot([], [], color='#ff5555', lw=1.0, label='Iu')
    line_iv, = ax_phase.plot([], [], color='#55ff55', lw=1.0, label='Iv')
    line_iw, = ax_phase.plot([], [], color='#5555ff', lw=1.0, label='Iw')
    ax_phase.legend(loc='upper right', fontsize=8)

    line_liss, = ax_liss.plot([], [], color='#55aaff', lw=0.8, alpha=0.7)
    dot_liss,  = ax_liss.plot([], [], 'o', color='#ffaa00', ms=5)

    # --- Buttons and slider ---
    ax_slider = fig.add_axes([0.08, 0.01, 0.30, 0.025])
    slider_throttle = Slider(ax_slider, 'Throttle', 0.0, 1.0,
                              valinit=0.0, valstep=0.001,
                              color='#55aaff')

    ax_btn_log = fig.add_axes([0.42, 0.005, 0.12, 0.035])
    btn_log = Button(ax_btn_log, 'Start Log', color=BTN_GREEN, hovercolor='#3d7a3d')

    ax_btn_stop = fig.add_axes([0.56, 0.005, 0.12, 0.035])
    btn_stop = Button(ax_btn_stop, 'Stop Motor', color=BTN_RED, hovercolor='#7a3d3d')

    ax_btn_reset = fig.add_axes([0.70, 0.005, 0.12, 0.035])
    btn_reset = Button(ax_btn_reset, 'Reset FC', color=BTN_RED, hovercolor='#7a3d3d')

    def on_slider(val):
        global g_throttle
        g_throttle = val
        if g_serial:
            send_throttle(g_serial, val)

    def on_start_log(event):
        if g_serial:
            send_log_class(g_serial, LOG_CLASS_CURRENT)
            print('  >> Start Log (Clarke)')

    def on_stop_motor(event):
        global g_throttle
        g_throttle = 0.0
        slider_throttle.set_val(0.0)
        if g_serial:
            send_throttle(g_serial, 0.0)
            send_log_class(g_serial, 0)

    def on_reset(event):
        if g_serial:
            send_reset(g_serial)

    slider_throttle.on_changed(on_slider)
    btn_log.on_clicked(on_start_log)
    btn_stop.on_clicked(on_stop_motor)
    btn_reset.on_clicked(on_reset)

    def update(frame):
        nonlocal h_ia, h_ib, h_iu, h_iv, h_iw

        count = 0
        latest = None
        while not data_queue.empty() and count < 20:
            latest = data_queue.get_nowait()
            count += 1

            h_ia  = np.roll(h_ia, -1);   h_ia[-1]  = latest[1]
            h_ib  = np.roll(h_ib, -1);   h_ib[-1]  = latest[2]
            h_iu  = np.roll(h_iu, -1);   h_iu[-1]  = latest[4]
            h_iv  = np.roll(h_iv, -1);   h_iv[-1]  = latest[5]
            h_iw  = np.roll(h_iw, -1);   h_iw[-1]  = latest[6]

        if latest is None:
            return

        state = int(latest[0])
        ia    = latest[1]
        ib    = latest[2]
        vbus  = latest[7]

        # Update Clarke traces
        x = np.arange(HISTORY_LEN)
        line_ia.set_data(x, h_ia)
        line_ib.set_data(x, h_ib)
        ax_clarke.set_xlim(0, HISTORY_LEN)

        # Update phase traces
        line_iu.set_data(x, h_iu)
        line_iv.set_data(x, h_iv)
        line_iw.set_data(x, h_iw)
        ax_phase.set_xlim(0, HISTORY_LEN)

        # Auto-scale Y (Clarke)
        all_curr = np.concatenate([h_ia[-100:], h_ib[-100:]])
        if np.any(all_curr != 0):
            ymax = max(abs(all_curr.max()), abs(all_curr.min()), 0.1) * 1.3
            ax_clarke.set_ylim(-ymax, ymax)

        # Auto-scale Y (Phase)
        all_phase = np.concatenate([h_iu[-100:], h_iv[-100:], h_iw[-100:]])
        if np.any(all_phase != 0):
            pymax = max(abs(all_phase.max()), abs(all_phase.min()), 0.1) * 1.3
            ax_phase.set_ylim(-pymax, pymax)

        # Update Lissajous
        n = min(100, HISTORY_LEN)
        line_liss.set_data(h_ia[-n:], h_ib[-n:])
        dot_liss.set_data([ia], [ib])
        if np.any(all_curr != 0):
            ax_liss.set_xlim(-ymax, ymax)
            ax_liss.set_ylim(-ymax, ymax)

        # Status text
        state_name = STATE_NAMES.get(state, '?')

        # Compute amplitude and phase quality from recent data
        ia_recent = h_ia[-100:]
        ib_recent = h_ib[-100:]
        ia_amp = (ia_recent.max() - ia_recent.min()) / 2.0
        ib_amp = (ib_recent.max() - ib_recent.min()) / 2.0

        iu_recent = h_iu[-100:]
        iv_recent = h_iv[-100:]
        iw_recent = h_iw[-100:]
        iu_amp = (iu_recent.max() - iu_recent.min()) / 2.0
        iv_amp = (iv_recent.max() - iv_recent.min()) / 2.0
        iw_amp = (iw_recent.max() - iw_recent.min()) / 2.0

        lines = [
            f'State: {state_name}',
            f'Vbus:  {vbus:.1f} V',
            f'',
            f'I\u03b1 amp: {ia_amp:.3f} A   I\u03b2 amp: {ib_amp:.3f} A',
        ]

        if ia_amp > 0.01 and ib_amp > 0.01:
            ratio = min(ia_amp, ib_amp) / max(ia_amp, ib_amp)
            lines.append(f'Amplitude ratio: {ratio:.2f}')
        else:
            ratio = 0

        lines.append('')
        lines.append(f'Iu amp: {iu_amp:.3f}   Iv amp: {iv_amp:.3f}   Iw amp: {iw_amp:.3f}')

        if ratio > 0.85:
            lines.append('\n\u2714 Amplitudes balanced (circle)')
        elif ratio > 0.7:
            lines.append('\n\u2714 Acceptable (mild ellipse)')
        elif ratio > 0:
            lines.append('\n\u26a0 Amplitudes unbalanced (ellipse)')

        info_text.set_text('\n'.join(lines))

        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=80)
    timer.add_callback(update, None)
    timer.start()

    plt.show()

    # Cleanup
    if g_serial:
        try:
            send_throttle(g_serial, 0.0)
            send_log_class(g_serial, 0)
        except Exception:
            pass


if __name__ == '__main__':
    main()
