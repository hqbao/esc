#!/usr/bin/env python3
"""Quick headless ESC 6-step test — send throttle, capture telemetry, report ZC status."""

import serial, serial.tools.list_ports, struct, time, sys

BAUD = 19200
DB_CMD_THROTTLE  = 0x01
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07
LOG_CLASS_COMMUTATION = 3
LOG_CLASS_NONE = 0

STATE_NAMES = {0: 'IDLE', 1: 'ALIGN', 2: 'RAMP', 3: 'CLOSEDLOOP'}
THROTTLE = 0.08   # 8%
TEST_DURATION = 8  # seconds

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
        self.cmd = 0; self.size = 0; self.payload = bytearray()
        self.ck_acc = 0; self.ck_lo = 0
    def feed(self, data):
        for b in data:
            if self.state == self.S1:
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

def main():
    port = find_port()
    if not port:
        print("ERROR: No serial port found"); sys.exit(1)
    print(f"Port: {port}")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()
    parser = FrameParser()

    # Reset FC first
    print("Resetting FC...")
    ser.write(build_frame(DB_CMD_RESET, b''))
    ser.flush()
    time.sleep(1.5)
    ser.reset_input_buffer()

    # Enable logging
    print(f"Enabling LOG_CLASS_COMMUTATION...")
    ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([LOG_CLASS_COMMUTATION])))
    ser.flush()
    time.sleep(0.3)

    # Send throttle
    print(f"Sending throttle={THROTTLE} ({THROTTLE*100:.0f}%)...")
    ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', THROTTLE)))
    ser.flush()

    t0 = time.time()
    count = 0
    last_state = -1
    max_zc = 0
    max_window = 0
    reached_closedloop = False

    print(f"\n{'t':>5s}  {'State':<12s}  {'Step':>4s}  {'eRPS':>7s}  {'Duty':>5s}  {'Vbus':>5s}  {'Period':>6s}  {'ZC_cnt':>6s}  {'Ramp_p':>6s}  {'Consec':>6s}  {'Miss':>4s}  {'Comp':>4s}  {'Map':>6s}  {'Win':>3s}")
    print("-" * 110)

    try:
        while time.time() - t0 < TEST_DURATION:
            raw = ser.read(256)
            if not raw:
                continue
            for cmd, payload in parser.feed(raw):
                if cmd == 0x00 and len(payload) >= 48:
                    f = struct.unpack('<12f', payload[:48])
                    state = int(f[0])
                    step = int(f[1])
                    speed = f[2]
                    duty = f[3]
                    vbus = f[4]
                    period = int(f[5])
                    zc_count = int(f[6])
                    ramp_period = int(f[7])
                    consec = int(f[8])
                    zc_miss = int(f[9])
                    comp_raw = int(f[10])
                    packed11 = int(f[11])
                    zc_map = packed11 & 0x3F
                    zc_window = (packed11 >> 8) & 0xFF

                    max_zc = max(max_zc, zc_count)
                    max_window = max(max_window, zc_window)

                    sname = STATE_NAMES.get(state, f'?{state}')
                    if state != last_state:
                        print(f"\n  >>> STATE CHANGE: {sname}")
                        last_state = state
                        if state == 3:
                            reached_closedloop = True

                    count += 1
                    if count % 3 == 0:  # Print every 3rd frame (~120ms)
                        elapsed = time.time() - t0
                        c1 = comp_raw & 1
                        c2 = (comp_raw >> 1) & 1
                        c4 = (comp_raw >> 2) & 1
                        print(f"{elapsed:5.1f}  {sname:<12s}  {step:4d}  {speed:7.2f}  {duty:5.2f}  {vbus:5.1f}  {period:6d}  {zc_count:6d}  {ramp_period:6d}  {consec:6d}  {zc_miss:4d}  {c1}{c2}{c4}  {zc_map:06b}  {zc_window:3d}")

        # Stop motor
        print(f"\n--- Stopping motor ---")
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0)))
        ser.flush()
        time.sleep(0.2)
        ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([LOG_CLASS_NONE])))
        ser.flush()

    except KeyboardInterrupt:
        print("\nInterrupted — stopping motor")
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0)))
        ser.flush()

    finally:
        ser.close()

    print(f"\n=== SUMMARY ===")
    print(f"  Frames received: {count}")
    print(f"  Max ZC count:    {max_zc}")
    print(f"  Max ZC window:   {max_window}/12")
    print(f"  Reached CLOSEDLOOP: {'YES' if reached_closedloop else 'NO'}")

    if max_zc == 0:
        print(f"\n  DIAGNOSIS: Zero ZC detections — comparators may not be triggering.")
        print(f"  Check comp_raw column: if always 000 or 111, DAC threshold or wiring issue.")
    elif not reached_closedloop:
        print(f"\n  DIAGNOSIS: Got ZCs but never reached CLOSEDLOOP.")
        print(f"  ZC window peaked at {max_window}/12 (need 10).")
    else:
        print(f"\n  SUCCESS: Motor reached closed-loop commutation!")

if __name__ == '__main__':
    main()
