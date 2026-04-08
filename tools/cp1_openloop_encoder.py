import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
import time

"""
CP1: Open-Loop + Encoder Verification

Runs motor in pure open-loop. Logs (LOG_CLASS_FOC = 1, 8 floats):
  [0] state          (IDLE=0, CALIBRATE=1, ALIGN=2, RAMP=3, OPENLOOP=4)
  [1] encoder_angle  (rad)
  [2] ol_angle       (open-loop commanded angle, rad)
  [3] omega_est      (observer speed, rad/s electrical)
  [4] amplitude      (PWM ticks)
  [5] encoder_offset (captured at align, rad)
  [6] theta_est      (observer angle, rad)
  [7] bemf_mag       (V)

PASS criteria:
  - Encoder angle tracks open-loop angle (green follows yellow)
  - Speed responds to throttle changes
  - Motor spins smoothly without stalling

Usage:
  python3 cp1_openloop_encoder.py

Safety:
  - Test with motor UNLOADED (no propeller)
  - Start with low throttle (< 0.10)
  - Emergency stop: click "Stop Motor" or close window
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 19200
SEND_LOG_ID = 0x00

# DB protocol constants
DB_CMD_THROTTLE  = 0x01
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07
LOG_CLASS_OPENLOOP = 6

# --- Dark Theme ---
BG_COLOR    = '#1e1e1e'
PANEL_COLOR = '#252526'
TEXT_COLOR   = '#cccccc'
DIM_TEXT     = '#888888'
GRID_COLOR   = '#3c3c3c'
BTN_GREEN    = '#2d5a2d'
BTN_GREEN_HOV = '#3d7a3d'
BTN_RED      = '#5a2d2d'
BTN_RED_HOV  = '#7a3d3d'

STATE_NAMES = {0: 'IDLE', 1: 'CALIBRATE', 2: 'ALIGN', 3: 'RAMP', 4: 'OPENLOOP', 5: 'CLOSEDLOOP'}
STATE_COLORS = {0: '#888888', 1: '#ffaa00', 2: '#ffaa00', 3: '#55aaff', 4: '#55ff55', 5: '#ff55ff'}

HISTORY_LEN = 200  # samples at 25Hz = 8 seconds

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
g_logging_active = False
g_current_throttle = 0.0


def build_db_frame(cmd_id, payload):
    """Build a DB protocol frame."""
    msg_class = 0x00
    length = len(payload)
    header = struct.pack('<2sBBH', b'db', cmd_id, msg_class, length)
    ck = cmd_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
    for b in payload:
        ck += b
    ck &= 0xFFFF
    return header + payload + struct.pack('<H', ck)


def send_throttle(ser, throttle):
    """Send throttle command (0.0-1.0) to ESC."""
    payload = struct.pack('<f', throttle)
    frame = build_db_frame(DB_CMD_THROTTLE, payload)
    ser.write(frame)
    ser.flush()


def send_log_class(ser, log_class):
    """Send log class selection."""
    payload = bytes([log_class])
    frame = build_db_frame(DB_CMD_LOG_CLASS, payload)
    ser.write(frame)
    ser.flush()
    print(f"  \u2192 Log class: {log_class}")


def send_reset(ser):
    """Send reset command."""
    payload = bytes([0x00])
    frame = build_db_frame(DB_CMD_RESET, payload)
    ser.write(frame)
    ser.flush()
    print("  \u2192 Reset command sent")


def serial_reader():
    """Background thread: reads DB frames from ESC, parses 8-float payloads."""
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

            hdr = ser.read(4)  # cmd, class, size_lo, size_hi
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


# --- GUI ---
def main():
    global g_logging_active, g_current_throttle

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

    fig = plt.figure(figsize=(14, 9))
    fig.canvas.manager.set_window_title('CP1: Open-Loop + Encoder Verification')

    # Layout: 2 rows — top full-width angle chart, bottom 3-column (speed, amplitude, status)
    gs = fig.add_gridspec(2, 3, height_ratios=[1, 1],
                          hspace=0.35, wspace=0.3,
                          left=0.08, right=0.95, top=0.95, bottom=0.10)

    ax_angle = fig.add_subplot(gs[0, :])     # encoder vs OL angle (full width)
    ax_speed = fig.add_subplot(gs[1, 0])     # speed
    ax_amp   = fig.add_subplot(gs[1, 1])     # amplitude + BEMF
    ax_info  = fig.add_subplot(gs[1, 2])     # status text

    # History arrays
    h_enc_angle = np.zeros(HISTORY_LEN)
    h_ol_angle  = np.zeros(HISTORY_LEN)
    h_speed     = np.zeros(HISTORY_LEN)
    h_amplitude = np.zeros(HISTORY_LEN)
    h_bemf      = np.zeros(HISTORY_LEN)
    x_axis = np.arange(HISTORY_LEN)

    # --- Top chart: Encoder vs Open-Loop angle ---
    ax_angle.set_title('Encoder (green) vs Open-Loop Commanded (yellow)', fontsize=10)
    ax_angle.set_ylabel('Angle (°)')
    ax_angle.set_ylim(-10, 370)
    ax_angle.grid(True, alpha=0.3)
    line_enc, = ax_angle.plot(x_axis, h_enc_angle, color='#55ff55', lw=1.5, label='Encoder')
    line_ol,  = ax_angle.plot(x_axis, h_ol_angle,  color='#ffdd44', lw=1.5, label='Open-Loop')
    ax_angle.legend(loc='upper right', fontsize=9)

    # --- Bottom left: Speed ---
    ax_speed.set_title('Speed (°/s)', fontsize=10)
    ax_speed.set_ylabel('°/s')
    ax_speed.grid(True, alpha=0.3)
    line_speed, = ax_speed.plot(x_axis, h_speed, color='#55aaff', lw=1.2)

    # --- Bottom center: Amplitude + BEMF ---
    ax_amp.set_title('PWM Amplitude / VBUS', fontsize=10)
    ax_amp.grid(True, alpha=0.3)
    line_amp, = ax_amp.plot(x_axis, h_amplitude, color='#ffaa55', lw=1.0, label='Amplitude')
    ax_amp2 = ax_amp.twinx()
    line_bemf, = ax_amp2.plot(x_axis, h_bemf, color='#ff55ff', lw=1.0, label='VBUS (V)')
    ax_amp.legend(loc='upper left', fontsize=8)
    ax_amp2.legend(loc='upper right', fontsize=8)
    ax_amp2.tick_params(axis='y', colors='#ff55ff')

    # --- Bottom right: Status panel ---
    ax_info.axis('off')
    ax_info.set_title('Status', fontsize=10)
    info_text = ax_info.text(0.05, 0.95, 'Waiting for data...',
                             fontsize=11, color=TEXT_COLOR,
                             va='top', family='monospace',
                             transform=ax_info.transAxes)

    # --- Slider + Buttons ---
    ax_slider = fig.add_axes([0.08, 0.02, 0.30, 0.025])
    slider = Slider(ax_slider, 'Throttle', 0.0, 1.0, valinit=0.0, valstep=0.001,
                    color='#55aaff')

    ax_btn_start = fig.add_axes([0.42, 0.015, 0.12, 0.035])
    ax_btn_stop  = fig.add_axes([0.56, 0.015, 0.12, 0.035])
    ax_btn_reset = fig.add_axes([0.70, 0.015, 0.12, 0.035])

    btn_start = Button(ax_btn_start, 'Start Log', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_stop  = Button(ax_btn_stop, 'Stop Motor', color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_reset = Button(ax_btn_reset, 'Reset ESC', color=BTN_RED, hovercolor=BTN_RED_HOV)

    def on_slider_change(val):
        global g_current_throttle
        g_current_throttle = val
        if g_serial and g_serial.is_open:
            send_throttle(g_serial, val)

    def on_start_log(event):
        global g_logging_active
        if g_serial and g_serial.is_open:
            send_log_class(g_serial, LOG_CLASS_OPENLOOP)
            g_logging_active = True

    def on_stop_motor(event):
        global g_current_throttle
        g_current_throttle = 0.0
        slider.set_val(0.0)
        if g_serial and g_serial.is_open:
            send_throttle(g_serial, 0.0)
            send_log_class(g_serial, 0)

    def on_reset(event):
        global g_current_throttle
        g_current_throttle = 0.0
        slider.set_val(0.0)
        if g_serial and g_serial.is_open:
            send_reset(g_serial)

    slider.on_changed(on_slider_change)
    btn_start.on_clicked(on_start_log)
    btn_stop.on_clicked(on_stop_motor)
    btn_reset.on_clicked(on_reset)

    def update(frame):
        nonlocal h_enc_angle, h_ol_angle, h_speed, h_amplitude, h_bemf

        updated = False
        last_vals = None

        while not data_queue.empty():
            vals = data_queue.get_nowait()
            last_vals = vals
            updated = True

            h_enc_angle = np.roll(h_enc_angle, -1);  h_enc_angle[-1] = np.degrees(vals[1])
            h_ol_angle  = np.roll(h_ol_angle, -1);   h_ol_angle[-1]  = np.degrees(vals[2])
            h_speed     = np.roll(h_speed, -1);       h_speed[-1]     = np.degrees(vals[3])
            h_amplitude = np.roll(h_amplitude, -1);    h_amplitude[-1] = vals[4]
            h_bemf      = np.roll(h_bemf, -1);        h_bemf[-1]      = vals[6]

        if not updated:
            return

        line_enc.set_ydata(h_enc_angle)
        line_ol.set_ydata(h_ol_angle)
        line_speed.set_ydata(h_speed)
        line_amp.set_ydata(h_amplitude)
        line_bemf.set_ydata(h_bemf)

        # Auto-scale
        for ax, data in [(ax_speed, h_speed)]:
            dmin, dmax = np.min(data), np.max(data)
            margin = max(abs(dmax - dmin) * 0.1, 0.5)
            ax.set_ylim(dmin - margin, dmax + margin)

        amp_min, amp_max = np.min(h_amplitude), np.max(h_amplitude)
        amp_margin = max(abs(amp_max - amp_min) * 0.1, 0.5)
        ax_amp.set_ylim(amp_min - amp_margin, amp_max + amp_margin)

        bemf_min, bemf_max = np.min(h_bemf), np.max(h_bemf)
        bemf_margin = max(abs(bemf_max - bemf_min) * 0.1, 0.5)
        ax_amp2.set_ylim(bemf_min - bemf_margin, bemf_max + bemf_margin)

        if last_vals:
            state_id = int(last_vals[0])
            state_name = STATE_NAMES.get(state_id, f'?{state_id}')
            state_color = STATE_COLORS.get(state_id, TEXT_COLOR)
            offset_deg = np.degrees(last_vals[5])

            info_text.set_text(
                f'State: {state_name}\n\n'
                f'Encoder:  {np.degrees(last_vals[1]):6.1f}\u00b0\n'
                f'OL cmd:   {np.degrees(last_vals[2]):6.1f}\u00b0\n'
                f'Offset:   {offset_deg:6.1f}\u00b0\n\n'
                f'Speed:    {np.degrees(last_vals[3]):6.0f} °/s\n'
                f'VBUS:     {last_vals[6]:6.2f} V\n\n'
                f'Throttle: {last_vals[7]:.3f}'
            )
            info_text.set_color(state_color)

        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=80)  # ~12 FPS
    timer.add_callback(update, None)
    timer.start()

    plt.show()

    # Cleanup - stop motor on exit
    if g_serial and g_serial.is_open:
        try:
            send_throttle(g_serial, 0.0)
            send_log_class(g_serial, 0)
            time.sleep(0.1)
        except Exception:
            pass


if __name__ == '__main__':
    main()
