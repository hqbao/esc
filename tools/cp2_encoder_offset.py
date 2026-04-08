import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import time
import math

"""
CP2: Encoder Offset Calibration

Verifies the encoder offset by locking the rotor at 4 known electrical
angles (0, 90, 180, 270 degrees) and reading the encoder at each.

PASS criteria:
  - Encoder readings step by ~90 deg electrical (pi/2 rad) between locks
  - All 4 steps are consistent within ±20 deg

Log format (LOG_CLASS_FOC = 1, 8 floats):
  [0] state          (0=IDLE, 1=CALIBRATE, 2=ALIGN, 3=RAMP, 4=OPENLOOP)
  [1] cal_step       (0-3 during calibration, 4 when done)
  [2] applied_angle  (rad, the field angle being applied)
  [3] enc_now        (rad, current encoder electrical angle)
  [4] enc@0          (rad, captured encoder at 0 deg lock)
  [5] enc@90         (rad, captured encoder at 90 deg lock)
  [6] enc@180        (rad, captured encoder at 180 deg lock)
  [7] enc@270        (rad, captured encoder at 270 deg lock)

Usage:
  python3 cp2_encoder_offset.py

Safety:
  - Motor is UNLOADED during calibration
  - Calibration applies gentle field (25% amplitude), no spinning
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 19200
SEND_LOG_ID = 0x00

DB_CMD_THROTTLE  = 0x01
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07
LOG_CLASS_FOC    = 1

# --- Dark Theme ---
BG_COLOR     = '#1e1e1e'
PANEL_COLOR  = '#252526'
TEXT_COLOR    = '#cccccc'
DIM_TEXT      = '#888888'
GRID_COLOR   = '#3c3c3c'
BTN_GREEN    = '#2d5a2d'
BTN_RED      = '#5a2d2d'
COLOR_PASS   = '#55ff55'
COLOR_FAIL   = '#ff5555'
COLOR_WAIT   = '#ffaa00'

STATE_NAMES = {
    0: 'IDLE', 1: 'CALIBRATE', 2: 'ALIGN',
    3: 'RAMP', 4: 'OPENLOOP', 5: 'CLOSEDLOOP'
}

HISTORY_LEN = 200
TWO_PI = 2.0 * math.pi

# --- Auto-detect serial port ---
ports = serial.tools.list_ports.comports()
print("Scanning for serial ports...")
for port, desc, hwid in sorted(ports):
    if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB']):
        SERIAL_PORT = port
        print(f"  \u2713 Auto-selected: {port} ({desc})")
        break
    else:
        print(f"  \u00b7 Skipped: {port} ({desc})")

if not SERIAL_PORT:
    print("  \u2717 No compatible serial port found.")

# --- Global State ---
data_queue = queue.Queue()
g_serial = None


def build_db_frame(cmd_id, payload):
    msg_class = 0x00
    length = len(payload)
    header = struct.pack('<2sBBH', b'db', cmd_id, msg_class, length)
    ck = cmd_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
    for b in payload:
        ck += b
    ck &= 0xFFFF
    return header + payload + struct.pack('<H', ck)


def send_log_class(ser, log_class):
    payload = bytes([log_class])
    frame = build_db_frame(DB_CMD_LOG_CLASS, payload)
    ser.write(frame)
    ser.flush()
    print(f"  \u2192 Log class: {log_class}")


def send_throttle(ser, throttle):
    payload = struct.pack('<f', throttle)
    frame = build_db_frame(DB_CMD_THROTTLE, payload)
    ser.write(frame)
    ser.flush()


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
            _ = ser.read(2)  # checksum

            if msg_id == SEND_LOG_ID and length == 32:
                vals = struct.unpack('<8f', payload)
                data_queue.put(vals)
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


def wrap_angle_diff(a, b):
    """Signed angular difference (a - b), wrapped to [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= TWO_PI
    while d < -math.pi:
        d += TWO_PI
    return d


def main():
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

    fig = plt.figure(figsize=(12, 8))
    fig.canvas.manager.set_window_title('CP2: Encoder Offset Calibration')

    gs = fig.add_gridspec(3, 2, height_ratios=[1.2, 1, 0.4],
                          hspace=0.45, wspace=0.35,
                          left=0.08, right=0.95, top=0.93, bottom=0.06)

    ax_enc    = fig.add_subplot(gs[0, :])   # real-time encoder angle
    ax_bar    = fig.add_subplot(gs[1, 0])   # bar chart: expected vs actual
    ax_result = fig.add_subplot(gs[1, 1])   # text results

    ax_enc.set_title('Encoder Electrical Angle (live)', fontsize=10)
    ax_enc.set_ylabel('Angle (rad)')
    ax_enc.set_ylim(-0.3, TWO_PI + 0.3)
    ax_enc.grid(True, alpha=0.3)

    ax_bar.set_title('Encoder Readings at 4 Lock Angles', fontsize=10)
    ax_bar.set_ylabel('Encoder (rad)')

    ax_result.axis('off')
    ax_result.set_title('Calibration Results', fontsize=10)

    # History
    h_enc    = np.zeros(HISTORY_LEN)
    h_field  = np.zeros(HISTORY_LEN)

    line_enc,  = ax_enc.plot([], [], color='#55ff55', lw=1.5, label='Encoder')
    line_field, = ax_enc.plot([], [], color='#ffaa00', lw=1.5, label='Applied field', ls='--')
    ax_enc.legend(loc='upper right', fontsize=8)

    # State
    cal_results = [None, None, None, None]
    cal_done = [False]
    result_text = ax_result.text(0.05, 0.9, 'Waiting for calibration...',
                                  fontsize=11, color=COLOR_WAIT,
                                  va='top', family='monospace',
                                  transform=ax_result.transAxes)

    # --- Buttons ---
    ax_btn_cal = fig.add_axes([0.08, 0.01, 0.18, 0.04])
    btn_cal = Button(ax_btn_cal, 'Start Calibration',
                     color=BTN_GREEN, hovercolor='#3d7a3d')

    ax_btn_reset = fig.add_axes([0.30, 0.01, 0.12, 0.04])
    btn_reset = Button(ax_btn_reset, 'Reset FC',
                       color=BTN_RED, hovercolor='#7a3d3d')

    ax_btn_stop = fig.add_axes([0.46, 0.01, 0.12, 0.04])
    btn_stop = Button(ax_btn_stop, 'Stop Log',
                      color='#2d2d5a', hovercolor='#3d3d7a')

    def on_calibrate(event):
        if g_serial:
            print('  >> Start Calibration clicked')
            cal_done[0] = False
            for i in range(4):
                cal_results[i] = None
            result_text.set_text('Calibrating...\nLocking rotor at 4 angles...')
            result_text.set_color(COLOR_WAIT)
            fig.canvas.draw_idle()
            # Force motor to IDLE first (in case previous tool left it running)
            send_throttle(g_serial, 0.0)
            send_log_class(g_serial, 0)
            time.sleep(0.15)
            # Now start calibration (LOG_CLASS_FOC while IDLE)
            send_log_class(g_serial, LOG_CLASS_FOC)
        else:
            print('  ✗ No serial connection!')
            result_text.set_text('ERROR: No serial port connected')
            result_text.set_color(COLOR_FAIL)
            fig.canvas.draw_idle()

    def on_reset(event):
        if g_serial:
            send_reset(g_serial)
        else:
            print('  ✗ No serial connection!')

    def on_stop(event):
        if g_serial:
            send_log_class(g_serial, 0)
        else:
            print('  ✗ No serial connection!')

    btn_cal.on_clicked(on_calibrate)
    btn_reset.on_clicked(on_reset)
    btn_stop.on_clicked(on_stop)

    def update(frame):
        nonlocal h_enc, h_field

        count = 0
        latest = None
        while not data_queue.empty() and count < 20:
            latest = data_queue.get_nowait()
            count += 1

            state    = int(latest[0])
            cal_step = int(latest[1])
            applied  = latest[2]
            enc_now  = latest[3]
            enc_0    = latest[4]
            enc_90   = latest[5]
            enc_180  = latest[6]
            enc_270  = latest[7]

            h_enc   = np.roll(h_enc, -1);   h_enc[-1]   = enc_now
            h_field = np.roll(h_field, -1);  h_field[-1] = applied

            # Check if calibration completed
            if state == 0 and cal_step >= 4 and not cal_done[0]:
                cal_results[0] = enc_0
                cal_results[1] = enc_90
                cal_results[2] = enc_180
                cal_results[3] = enc_270
                cal_done[0] = True
            elif state == 1:
                # During calibration, show live captured values
                if enc_0 != 0.0 or cal_step > 0:
                    cal_results[0] = enc_0
                if enc_90 != 0.0 or cal_step > 1:
                    cal_results[1] = enc_90
                if enc_180 != 0.0 or cal_step > 2:
                    cal_results[2] = enc_180
                if enc_270 != 0.0 or cal_step > 3:
                    cal_results[3] = enc_270

        if latest is None:
            return

        # Update encoder trace
        x = np.arange(HISTORY_LEN)
        line_enc.set_data(x, h_enc)
        line_field.set_data(x, h_field)
        ax_enc.set_xlim(0, HISTORY_LEN)

        # Update bar chart
        ax_bar.clear()
        ax_bar.set_title('Encoder Readings at 4 Lock Angles', fontsize=10)
        ax_bar.set_ylabel('Encoder (rad)')

        labels = ['0\u00b0', '90\u00b0', '180\u00b0', '270\u00b0']
        expected_offsets = [0, math.pi/2, math.pi, 3*math.pi/2]

        if cal_results[0] is not None:
            vals = []
            for i in range(4):
                v = cal_results[i] if cal_results[i] is not None else 0
                vals.append(v)

            bars = ax_bar.bar(labels, vals, color='#55aaff', width=0.5)

            # Draw expected angles (offset + 0, +pi/2, +pi, +3pi/2)
            if cal_done[0]:
                offset = vals[0]
                for i, eo in enumerate(expected_offsets):
                    expected = (offset + eo) % TWO_PI
                    ax_bar.plot([i - 0.3, i + 0.3], [expected, expected],
                               color=COLOR_PASS, lw=2, ls='--')
                ax_bar.plot([], [], color=COLOR_PASS, ls='--', label='Expected')
                ax_bar.legend(fontsize=8)

            ax_bar.set_ylim(0, TWO_PI + 0.5)
            ax_bar.axhline(y=TWO_PI, color=GRID_COLOR, ls=':', lw=0.5)

        # Update results text
        if cal_done[0] and all(r is not None for r in cal_results):
            offset = cal_results[0]
            lines = [f'Offset (enc@0\u00b0): {offset:.4f} rad ({math.degrees(offset):.1f}\u00b0)', '']

            diffs = []
            pass_all = True
            for i in range(1, 4):
                expected_rad = expected_offsets[i]
                actual_diff = wrap_angle_diff(cal_results[i], offset)
                if actual_diff < 0:
                    actual_diff += TWO_PI
                error = abs(wrap_angle_diff(actual_diff, expected_rad))
                error_deg = math.degrees(error)
                ok = error_deg < 20.0
                if not ok:
                    pass_all = False
                status = '\u2713' if ok else '\u2717'
                lines.append(
                    f'{labels[i]:>4s}: enc={cal_results[i]:.3f}  '
                    f'diff={math.degrees(actual_diff):.1f}\u00b0  '
                    f'expect={math.degrees(expected_rad):.1f}\u00b0  '
                    f'err={error_deg:.1f}\u00b0 {status}'
                )
                diffs.append(actual_diff)

            lines.append('')
            if pass_all:
                lines.append('\u2714 CP2 PASSED - Offset is consistent')
                result_text.set_color(COLOR_PASS)
            else:
                lines.append('\u2718 CP2 FAILED - Readings not consistent')
                result_text.set_color(COLOR_FAIL)

            # Check encoder direction
            if len(diffs) >= 2:
                if diffs[0] > 0 and diffs[1] > diffs[0]:
                    lines.append('  Encoder direction: POSITIVE (same as field)')
                else:
                    lines.append('  Encoder direction: CHECK carefully!')

            result_text.set_text('\n'.join(lines))
        elif not cal_done[0] and latest is not None and int(latest[0]) == 1:
            step = int(latest[1])
            step_label = ['0\u00b0', '90\u00b0', '180\u00b0', '270\u00b0']
            lbl = step_label[step] if step < 4 else 'done'
            result_text.set_text(f'Calibrating... step {step+1}/4 ({lbl})\n'
                                 f'enc_now = {latest[3]:.4f} rad')
            result_text.set_color(COLOR_WAIT)

        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=80)
    timer.add_callback(update, None)
    timer.start()

    plt.show()

    # Cleanup
    if g_serial:
        try:
            send_log_class(g_serial, 0)
        except Exception:
            pass


if __name__ == '__main__':
    main()
