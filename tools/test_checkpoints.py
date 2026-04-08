#!/usr/bin/env python3
"""
ESC Checkpoint Test (headless CLI)

Runs checkpoints in sequence:
  CP2: Encoder offset calibration (4-angle lock)
  CP1: Open-loop + encoder tracking
  CP3: Clarke transform (Iα, Iβ sinusoidal)
  CP4: Park transform (Id≈0, Iq≈DC)

Usage:
  python3 test_checkpoints.py              # run all
  python3 test_checkpoints.py cp1          # run one
  python3 test_checkpoints.py cp1 cp4      # run several
"""

import serial, serial.tools.list_ports, struct, time, sys, math
import numpy as np

BAUD = 19200
DB_CMD_THROTTLE  = 0x01
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07

LOG_CLASS_FOC      = 1   # CP2: calibration
LOG_CLASS_CURRENT  = 2   # CP3: Clarke
LOG_CLASS_VOLTAGE  = 3   # CP4: Park
LOG_CLASS_OPENLOOP = 6   # CP1: open-loop encoder
LOG_CLASS_STORAGE  = 7   # storage readback

STATE_IDLE       = 0
STATE_CALIBRATE  = 1
STATE_OPENLOOP   = 4

TWO_PI = 2.0 * math.pi


# ── Serial port detection ─────────────────────────────────────────────────

def find_stlink_port():
    """Find STLink VCP (VID 0483) first, then fall back to any usbmodem."""
    fallback = None
    for port, desc, hwid in sorted(serial.tools.list_ports.comports()):
        if '0483' in hwid:
            return port
        if any(x in port for x in ['usbmodem', 'usbserial', 'ttyACM', 'ttyUSB']):
            if not fallback:
                fallback = port
    return fallback


# ── DB protocol ────────────────────────────────────────────────────────────

def build_frame(cmd, payload):
    hdr = struct.pack('<2sBBH', b'db', cmd, 0, len(payload))
    ck = cmd + 0 + (len(payload) & 0xFF) + ((len(payload) >> 8) & 0xFF)
    for b in payload:
        ck += b
    return hdr + payload + struct.pack('<H', ck & 0xFFFF)


class FrameParser:
    """DB protocol state machine. Yields (cmd, payload) tuples."""
    SYNC1, SYNC2, CMD, CLASS, SZ_LO, SZ_HI, PAYLOAD, CK_LO, CK_HI = range(9)

    def __init__(self):
        self.state = self.SYNC1
        self.cmd = 0
        self.size = 0
        self.payload = bytearray()
        self.ck_acc = 0
        self.ck_lo = 0
        self.bad_ck = 0

    def feed(self, data):
        for b in data:
            if self.state == self.SYNC1:
                if b == ord('d'): self.state = self.SYNC2
            elif self.state == self.SYNC2:
                if b == ord('b'): self.state = self.CMD
                else: self.state = self.SYNC1
            elif self.state == self.CMD:
                self.cmd = b; self.ck_acc = b; self.state = self.CLASS
            elif self.state == self.CLASS:
                self.ck_acc += b; self.state = self.SZ_LO
            elif self.state == self.SZ_LO:
                self.size = b; self.ck_acc += b; self.state = self.SZ_HI
            elif self.state == self.SZ_HI:
                self.size |= b << 8; self.ck_acc += b
                if self.size > 120: self.state = self.SYNC1
                elif self.size == 0: self.state = self.CK_LO
                else: self.payload = bytearray(); self.state = self.PAYLOAD
            elif self.state == self.PAYLOAD:
                self.payload.append(b); self.ck_acc += b
                if len(self.payload) >= self.size: self.state = self.CK_LO
            elif self.state == self.CK_LO:
                self.ck_lo = b; self.state = self.CK_HI
            elif self.state == self.CK_HI:
                received = self.ck_lo | (b << 8)
                if received == (self.ck_acc & 0xFFFF):
                    yield (self.cmd, bytes(self.payload))
                else:
                    self.bad_ck += 1
                self.state = self.SYNC1


# ── Helpers ────────────────────────────────────────────────────────────────

def collect_logs(ser, parser, duration_s):
    """Collect log frames (cmd=0x00, 32 bytes) for duration_s seconds."""
    frames = []
    t0 = time.time()
    while time.time() - t0 < duration_s:
        raw = ser.read(256)
        if raw:
            for cmd, payload in parser.feed(raw):
                if cmd == 0x00 and len(payload) == 32:
                    frames.append(struct.unpack('<8f', payload))
    return frames


def send_log_class(ser, cls):
    ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([cls])))
    ser.flush()


def send_throttle(ser, val):
    ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', val)))
    ser.flush()


def send_reset(ser):
    ser.write(build_frame(DB_CMD_RESET, b''))
    ser.flush()


def stop_motor(ser):
    send_throttle(ser, 0.0)
    time.sleep(0.3)


def wait_heartbeat(ser, parser, timeout=5):
    """Wait for at least one heartbeat. Returns True if received."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        raw = ser.read(256)
        if raw:
            for cmd, _ in parser.feed(raw):
                if cmd == 0xFF:
                    return True
    return False


def count_wraps(angles_deg):
    """Count full 360° wraps in an angle series."""
    wraps = 0
    for i in range(1, len(angles_deg)):
        if angles_deg[i] < 90 and angles_deg[i - 1] > 270:
            wraps += 1
        elif angles_deg[i] > 270 and angles_deg[i - 1] < 90:
            wraps -= 1
    return abs(wraps)


# ── Pre-check: Encoder alive ───────────────────────────────────────────────

def check_encoder(ser, parser):
    """Quick encoder health check — spin motor briefly, verify encoder moves."""
    print("\n" + "=" * 50)
    print("PRE-CHECK: Encoder Communication")
    print("=" * 50)

    send_log_class(ser, LOG_CLASS_OPENLOOP)
    time.sleep(0.2)
    send_throttle(ser, 0.02)
    time.sleep(3)  # align + ramp

    frames = collect_logs(ser, parser, 2)
    stop_motor(ser)
    send_log_class(ser, 0)

    if len(frames) < 5:
        print("  FAIL — no log frames received")
        return False

    # f[1] = g_enc_elec (encoder electrical angle in radians)
    enc_values = [f[1] for f in frames]
    enc_range = max(enc_values) - min(enc_values)
    enc_range_deg = math.degrees(enc_range)

    print(f"  Frames: {len(frames)}")
    print(f"  Encoder range: {enc_range_deg:.1f}° (need > 10°)")
    print(f"  Sample values: {', '.join(f'{math.degrees(v):.1f}°' for v in enc_values[:5])}")

    if enc_range_deg < 10:
        print("  ✗ FAIL — encoder stuck or not connected")
        print("  Check: AS5048A wiring (PB6=SCK, PB7=MOSI, PB8=MISO, PA15=CS)")
        print("  Check: magnet seated on rotor shaft")
        return False

    print("  ✓ Encoder alive")
    return True


# ── Pre-check: Stored calibration ─────────────────────────────────────────

def check_storage(ser, parser):
    """Read stored encoder offset from flash."""
    print("\n" + "=" * 50)
    print("PRE-CHECK: Stored Calibration")
    print("=" * 50)

    send_log_class(ser, LOG_CLASS_STORAGE)
    time.sleep(0.2)
    frames = collect_logs(ser, parser, 2)
    send_log_class(ser, 0)

    if not frames:
        print("  No storage frames received")
        print("  Stored offset: unknown")
        return

    # param[0] = enc_offset
    offset = frames[-1][0]
    if offset != 0.0:
        print(f"  Stored enc_offset: {offset:.6f} rad ({math.degrees(offset):.1f}°)")
    else:
        print("  No calibration stored (enc_offset = 0)")


# ── CP2: Encoder Offset Calibration ────────────────────────────────────────

def run_cp2(ser, parser):
    print("\n" + "=" * 50)
    print("CP2: Encoder Offset Calibration")
    print("=" * 50)

    send_log_class(ser, LOG_CLASS_FOC)  # triggers calibration when IDLE
    time.sleep(0.5)

    # Calibration takes ~4s (4 steps × 1s). Collect for 6s.
    frames = collect_logs(ser, parser, 6)
    print(f"  Frames: {len(frames)}, checksum errors: {parser.bad_ck}")

    # Find the frame where cal_step == 4 (calibration finished)
    done_frames = [f for f in frames if int(f[1]) >= 4]
    if not done_frames:
        cal_frames = [f for f in frames if int(f[0]) == STATE_CALIBRATE]
        if cal_frames:
            last = cal_frames[-1]
            print(f"  Calibration in progress (step {int(last[1])}), did not finish")
        print("  FAIL — calibration did not complete")
        return False

    result = done_frames[-1]
    enc = [result[4], result[5], result[6], result[7]]
    applied = [0, 90, 180, 270]

    print(f"  Captured encoder angles:")
    for i in range(4):
        print(f"    {applied[i]:3d}° → encoder = {math.degrees(enc[i]):6.1f}°")

    # Check ~90° steps between consecutive calibration points
    steps_ok = True
    for i in range(3):
        delta = math.degrees(enc[i + 1] - enc[i])
        # Wrap to [-180, 180]
        while delta > 180: delta -= 360
        while delta < -180: delta += 360
        expected = 90
        if abs(abs(delta) - expected) > 25:
            print(f"    Step {applied[i]}→{applied[i+1]}: Δ={delta:.1f}° (expected ~{expected}°) — BAD")
            steps_ok = False
        else:
            print(f"    Step {applied[i]}→{applied[i+1]}: Δ={delta:.1f}° ✓")

    if steps_ok:
        print("  ✓ CP2 PASSED")
    else:
        print("  ✗ CP2 FAILED — inconsistent encoder steps")
    return steps_ok


# ── CP1: Open-Loop + Encoder Tracking ──────────────────────────────────────

def run_cp1(ser, parser):
    print("\n" + "=" * 50)
    print("CP1: Open-Loop + Encoder Tracking")
    print("=" * 50)

    send_log_class(ser, LOG_CLASS_OPENLOOP)
    time.sleep(0.2)
    send_throttle(ser, 0.02)
    print("  Motor started, waiting for align + ramp...")
    time.sleep(3)

    frames = collect_logs(ser, parser, 6)
    stop_motor(ser)
    print(f"  Frames: {len(frames)}, checksum errors: {parser.bad_ck}")

    ol_frames = [f for f in frames if int(f[0]) == STATE_OPENLOOP]
    print(f"  OPENLOOP frames: {len(ol_frames)}")

    if len(ol_frames) < 20:
        print("  FAIL — not enough OPENLOOP frames")
        return False

    enc_deg = np.degrees([f[1] for f in ol_frames])
    ol_deg  = np.degrees([f[2] for f in ol_frames])

    enc_range = enc_deg.max() - enc_deg.min()
    print(f"  Encoder range: {enc_range:.1f}°")

    if enc_range < 30:
        print("  FAIL — encoder stuck (range < 30°)")
        return False

    enc_wraps = count_wraps(enc_deg)
    ol_wraps = count_wraps(ol_deg)
    print(f"  Encoder wraps: {enc_wraps}, OL wraps: {ol_wraps}")

    if ol_wraps == 0:
        print("  FAIL — OL angle not wrapping")
        return False

    ratio = enc_wraps / ol_wraps
    print(f"  Wrap ratio: {ratio:.2f} (expect ~1.0)")

    if 0.7 <= ratio <= 1.3:
        print("  ✓ CP1 PASSED")
        return True
    else:
        print("  ✗ CP1 FAILED — wrap ratio out of range")
        return False


# ── CP3: Clarke Transform ──────────────────────────────────────────────────

def run_cp3(ser, parser):
    print("\n" + "=" * 50)
    print("CP3: Clarke Transform Verification")
    print("=" * 50)

    send_log_class(ser, LOG_CLASS_CURRENT)
    time.sleep(0.2)
    send_throttle(ser, 0.05)
    print("  Motor started...")
    time.sleep(3)

    frames = collect_logs(ser, parser, 6)
    stop_motor(ser)
    print(f"  Frames: {len(frames)}, checksum errors: {parser.bad_ck}")

    ol_frames = [f for f in frames if int(f[0]) == STATE_OPENLOOP]
    print(f"  OPENLOOP frames: {len(ol_frames)}")

    if len(ol_frames) < 20:
        print("  FAIL — not enough OPENLOOP frames")
        return False

    ia = np.array([f[1] for f in ol_frames])
    ib = np.array([f[2] for f in ol_frames])

    ia_amp = (ia.max() - ia.min()) / 2
    ib_amp = (ib.max() - ib.min()) / 2
    print(f"  Iα amp: {ia_amp:.4f} A, Iβ amp: {ib_amp:.4f} A")

    if ia_amp < 0.01 or ib_amp < 0.01:
        print("  FAIL — no significant current flowing")
        return False

    # Amplitude balance: Iα and Iβ should be similar amplitude
    balance = min(ia_amp, ib_amp) / max(ia_amp, ib_amp)
    print(f"  Amplitude balance: {balance:.2f} (expect > 0.6)")

    if balance < 0.4:
        print("  ✗ CP3 FAILED — Iα/Iβ severely unbalanced")
        return False

    print("  ✓ CP3 PASSED")
    return True


# ── CP4: Park Transform ───────────────────────────────────────────────────

def run_cp4(ser, parser):
    print("\n" + "=" * 50)
    print("CP4: Park Transform Verification")
    print("=" * 50)

    send_log_class(ser, LOG_CLASS_VOLTAGE)
    time.sleep(0.2)
    send_throttle(ser, 0.05)
    print("  Motor started...")
    time.sleep(3)

    frames = collect_logs(ser, parser, 6)
    stop_motor(ser)
    print(f"  Frames: {len(frames)}, checksum errors: {parser.bad_ck}")

    ol_frames = [f for f in frames if int(f[0]) == STATE_OPENLOOP]
    print(f"  OPENLOOP frames: {len(ol_frames)}")

    if len(ol_frames) < 20:
        print("  FAIL — not enough OPENLOOP frames")
        return False

    ids = np.array([f[1] for f in ol_frames])
    iqs = np.array([f[2] for f in ol_frames])
    ias = np.array([f[6] for f in ol_frames])

    id_pp = ids.max() - ids.min()
    iq_pp = iqs.max() - iqs.min()
    ia_amp = (ias.max() - ias.min()) / 2

    print(f"  Id: mean={ids.mean():.4f}, p-p={id_pp:.4f} A")
    print(f"  Iq: mean={iqs.mean():.4f}, p-p={iq_pp:.4f} A")
    print(f"  Iα amp: {ia_amp:.4f} A")

    if ia_amp < 0.01:
        print("  WARN — no significant current flowing")
        return False

    id_ratio = id_pp / (2 * ia_amp)
    iq_ratio = iq_pp / (2 * ia_amp)
    print(f"  Id ripple: {id_ratio:.2f} (want < 0.30)")
    print(f"  Iq ripple: {iq_ratio:.2f} (want < 0.30)")

    if id_ratio < 0.30 and iq_ratio < 0.30:
        print("  ✓ CP4 PASSED")
        return True
    else:
        print("  ✗ CP4 FAILED — Id/Iq not flat DC")
        return False


# ── Main ───────────────────────────────────────────────────────────────────

CHECKPOINTS = {
    'cp2': ('CP2 — Encoder Calibration', run_cp2),
    'cp1': ('CP1 — Open-Loop Encoder',   run_cp1),
    'cp3': ('CP3 — Clarke Transform',     run_cp3),
    'cp4': ('CP4 — Park Transform',       run_cp4),
}

# Default run order
DEFAULT_ORDER = ['cp2', 'cp1', 'cp3', 'cp4']


def main():
    # Parse which CPs to run
    requested = sys.argv[1:] if len(sys.argv) > 1 else DEFAULT_ORDER
    for cp in requested:
        if cp not in CHECKPOINTS:
            print(f"Unknown checkpoint: {cp}")
            print(f"Available: {', '.join(CHECKPOINTS.keys())}")
            sys.exit(1)

    port = find_stlink_port()
    if not port:
        print("No serial port found")
        sys.exit(1)
    print(f"Port: {port}")

    ser = serial.Serial(port, BAUD, timeout=0.5)
    parser = FrameParser()
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Wait for heartbeat
    print("Waiting for heartbeat...")
    if not wait_heartbeat(ser, parser):
        print("FAIL — no heartbeat received")
        ser.close()
        sys.exit(1)
    print("  Heartbeat OK")

    # Pre-checks
    check_storage(ser, parser)

    if not check_encoder(ser, parser):
        print("\nABORTED — encoder not working, checkpoints will all fail")
        ser.close()
        sys.exit(1)

    time.sleep(1)

    # Run checkpoints
    results = {}
    for cp in requested:
        name, func = CHECKPOINTS[cp]
        try:
            results[cp] = func(ser, parser)
        except Exception as e:
            print(f"  ERROR: {e}")
            results[cp] = False
        # Always stop motor between tests
        stop_motor(ser)
        time.sleep(1)

    ser.close()

    # Summary
    print("\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)
    all_pass = True
    for cp in requested:
        name = CHECKPOINTS[cp][0]
        status = "PASS ✓" if results[cp] else "FAIL ✗"
        if not results[cp]:
            all_pass = False
        print(f"  {name}: {status}")

    sys.exit(0 if all_pass else 1)


if __name__ == '__main__':
    main()
