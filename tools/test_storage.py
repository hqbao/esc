#!/usr/bin/env python3
"""
ESC Local Storage Test (headless CLI)

Verifies that encoder offset calibration survives power cycle:
  1. Check if a saved offset already exists (read storage log)
  2. Run calibration (writes offset to flash)
  3. Read offset via storage log class
  4. Reset MCU (simulates power cycle)
  5. Read offset again — must match pre-reset value

Usage:
  python3 test_storage.py
"""

import serial, serial.tools.list_ports, struct, time, sys

BAUD = 19200
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07

LOG_CLASS_FOC     = 1   # triggers calibration when IDLE
LOG_CLASS_STORAGE = 7   # reads back stored params


# ── Serial port detection ─────────────────────────────────────────────────

def find_stlink_port():
    """Find STLink VCP (VID 0483)."""
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

def send_log_class(ser, cls):
    ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([cls])))
    ser.flush()


def send_reset(ser):
    ser.write(build_frame(DB_CMD_RESET, b''))
    ser.flush()


def wait_heartbeat(ser, parser, timeout=5):
    t0 = time.time()
    while time.time() - t0 < timeout:
        raw = ser.read(256)
        if raw:
            for cmd, _ in parser.feed(raw):
                if cmd == 0xFF:
                    return True
    return False


def collect_logs(ser, parser, duration_s):
    """Collect log frames (cmd=0x00, 32 bytes = 8 floats)."""
    frames = []
    t0 = time.time()
    while time.time() - t0 < duration_s:
        raw = ser.read(256)
        if raw:
            for cmd, payload in parser.feed(raw):
                if cmd == 0x00 and len(payload) == 32:
                    frames.append(struct.unpack('<8f', payload))
    return frames


def read_storage_offset(ser, parser):
    """Request storage log class and read back enc_offset (param[0])."""
    send_log_class(ser, LOG_CLASS_STORAGE)
    time.sleep(0.2)  # wait for at least one 1Hz log
    frames = collect_logs(ser, parser, 2.0)
    send_log_class(ser, 0)  # stop logging
    if frames:
        return frames[-1][0]  # param[0] = enc_offset
    return None


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    port = find_stlink_port()
    if not port:
        print("ERROR: No STLink serial port found")
        sys.exit(1)

    print(f"Port: {port}")
    ser = serial.Serial(port, BAUD, timeout=0.1)
    parser = FrameParser()

    # Wait for MCU heartbeat
    print("Waiting for heartbeat...", end=" ", flush=True)
    if not wait_heartbeat(ser, parser):
        print("TIMEOUT")
        sys.exit(1)
    print("OK")

    # ── Step 1: Read current stored offset ──────────────────────────────
    print("\n── Step 1: Read pre-existing offset ──")
    offset_before = read_storage_offset(ser, parser)
    if offset_before is not None and offset_before != 0.0:
        print(f"  Stored offset: {offset_before:.6f} rad")
    else:
        print("  No valid offset stored (0.0 or not readable)")

    # ── Step 2: Run calibration ─────────────────────────────────────────
    print("\n── Step 2: Run encoder offset calibration ──")
    print("  Sending LOG_CLASS_FOC to trigger calibration...")
    send_log_class(ser, LOG_CLASS_FOC)
    time.sleep(0.5)

    # Calibration: 4 angles × 1s each = ~4s. Collect for 6s.
    frames = collect_logs(ser, parser, 6.0)
    print(f"  Collected {len(frames)} frames")

    # Check calibration completed: state==0 (IDLE) after cal
    done = [f for f in frames if int(f[0]) == 0 and int(f[1]) >= 4]
    if not done:
        print("  WARNING: Calibration may not have completed")
        # Try to read offset anyway
    else:
        cal_offset = done[-1][4]  # enc@0 from CP2 log
        print(f"  Calibration done. enc@0 = {cal_offset:.6f} rad")

    # ── Step 3: Read stored offset (post-calibration) ───────────────────
    print("\n── Step 3: Read offset after calibration ──")
    time.sleep(1.5)  # wait for 1Hz flush to flash
    offset_after_cal = read_storage_offset(ser, parser)
    if offset_after_cal is None or offset_after_cal == 0.0:
        print("  FAIL: Offset not saved to storage")
        sys.exit(1)
    print(f"  Stored offset: {offset_after_cal:.6f} rad")

    # ── Step 4: Reset MCU ───────────────────────────────────────────────
    print("\n── Step 4: Reset MCU ──")
    send_reset(ser)
    ser.close()
    time.sleep(2.0)  # wait for reset + boot

    # Reconnect
    ser = serial.Serial(port, BAUD, timeout=0.1)
    parser = FrameParser()
    print("  Waiting for heartbeat after reset...", end=" ", flush=True)
    if not wait_heartbeat(ser, parser):
        print("TIMEOUT")
        sys.exit(1)
    print("OK")

    # ── Step 5: Read offset after reset ─────────────────────────────────
    print("\n── Step 5: Read offset after reset ──")
    offset_after_reset = read_storage_offset(ser, parser)
    if offset_after_reset is None:
        print("  FAIL: Could not read storage after reset")
        ser.close()
        sys.exit(1)

    print(f"  Stored offset: {offset_after_reset:.6f} rad")

    # ── Verdict ─────────────────────────────────────────────────────────
    print("\n" + "=" * 50)
    match = abs(offset_after_cal - offset_after_reset) < 1e-6
    if match and offset_after_reset != 0.0:
        print(f"PASS: Offset persisted across reset")
        print(f"  Before reset: {offset_after_cal:.6f} rad")
        print(f"  After reset:  {offset_after_reset:.6f} rad")
    else:
        print(f"FAIL: Offset mismatch or zero")
        print(f"  Before reset: {offset_after_cal:.6f} rad")
        print(f"  After reset:  {offset_after_reset:.6f} rad")

    print("=" * 50)
    ser.close()
    sys.exit(0 if match else 1)


if __name__ == '__main__':
    main()
