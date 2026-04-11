#!/usr/bin/env python3
"""CLI test script — sends throttle, reads telemetry, prints ZC diagnostics."""

import serial, struct, threading, queue, time, sys

BAUD = 19200
DB_CMD_THROTTLE  = 0x01
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07
LOG_CLASS_COMMUTATION = 3
THROTTLE = 0.20


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


STATE_NAMES = {0: 'IDLE', 1: 'ALIGN', 2: 'RAMP', 3: 'CLOSEDLOOP'}


def find_port():
    import serial.tools.list_ports
    for p, d, h in sorted(serial.tools.list_ports.comports()):
        if 'usbmodem31203' in p:
            return p
    for p, d, h in sorted(serial.tools.list_ports.comports()):
        if any(x in p for x in ['usbmodem', 'usbserial', 'ttyACM', 'ttyUSB']):
            return p
    return None


def main():
    port = find_port()
    if not port:
        print("No serial port found"); sys.exit(1)
    print(f"Port: {port}")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    q = queue.Queue(maxsize=500)
    stop_evt = threading.Event()
    parser = FrameParser()

    def reader():
        while not stop_evt.is_set():
            try:
                raw = ser.read(256)
            except: break
            if raw:
                for cmd, payload in parser.feed(raw):
                    q.put((cmd, payload))

    threading.Thread(target=reader, daemon=True).start()

    # Reset MCU first to ensure clean state
    print("Resetting MCU...")
    ser.write(build_frame(DB_CMD_RESET, b''))
    ser.flush()
    time.sleep(2)
    ser.reset_input_buffer()

    # Start: send log class, wait, then throttle
    print("Sending start sequence...")
    ser.write(build_frame(DB_CMD_LOG_CLASS, bytes([LOG_CLASS_COMMUTATION])))
    ser.flush()
    time.sleep(0.5)
    ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', THROTTLE)))
    ser.flush()

    print(f"Throttle = {THROTTLE}, reading telemetry for 12 seconds...\n")
    print(f"{'Time':>5s}  {'State':>10s}  {'Step':>4s}  {'C1':>2s} {'C2':>2s} {'C4':>2s}  "
          f"{'ZCmap':>6s}  {'ZCwin':>5s}  {'ZCcnt':>5s}  {'Period':>6s}  {'DAC':>5s}  {'Vbus':>5s}")
    print("-" * 90)

    t0 = time.time()
    reached_cl = False
    last_print = 0

    try:
        while time.time() - t0 < 6:
            try:
                cmd, payload = q.get(timeout=0.1)
            except queue.Empty:
                continue

            if cmd == 0x00 and len(payload) >= 48:
                f = struct.unpack('<12f', payload[:48])
                state = int(f[0])
                step = int(f[1])
                erps = f[2]
                duty = f[3]
                vbus = f[4]
                step_period = f[5]
                zc_count = int(f[6])
                ramp_period = int(f[7])
                dac_val = int(f[8])
                zc_miss = int(f[9])
                step_zc_map = int(f[11]) & 0x3F
                zc_window_sum = (int(f[11]) >> 8) & 0xFF
                comp_raw = int(f[10]) & 0x07
                c1 = comp_raw & 1
                c2 = (comp_raw >> 1) & 1
                c4 = (comp_raw >> 2) & 1

                now = time.time() - t0
                if now - last_print >= 0.3:
                    last_print = now
                    sname = STATE_NAMES.get(state, f'?{state}')
                    map_bin = f'{step_zc_map:06b}'
                    print(f"{now:5.1f}  {sname:>10s}  {step:4d}  {c1:>2d} {c2:>2d} {c4:>2d}  "
                          f"{map_bin:>6s}  {zc_window_sum:3d}/12  {zc_count:5d}  "
                          f"{step_period:6.0f}  {dac_val:5d}  {vbus:5.1f}")

                    if state == 3 and not reached_cl:
                        reached_cl = True
                        print("\n*** CLOSED-LOOP REACHED! ***\n")

    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping motor...")
        ser.write(build_frame(DB_CMD_THROTTLE, struct.pack('<f', 0.0)))
        ser.flush()
        time.sleep(0.3)
        stop_evt.set()
        ser.close()

    if reached_cl:
        print("\nRESULT: SUCCESS — Motor reached closed-loop state")
    else:
        print("\nRESULT: FAILED — Motor did not reach closed-loop")
    return 0 if reached_cl else 1


if __name__ == '__main__':
    sys.exit(main())
