#!/usr/bin/env python3
"""Quick raw serial debug — see what bytes come back from ESC."""
import serial, struct, time

BAUD = 19200
port = '/dev/cu.usbmodem31203'

def build_frame(cmd, payload):
    hdr = struct.pack('<2sBBH', b'db', cmd, 0, len(payload))
    ck = cmd + 0 + (len(payload) & 0xFF) + ((len(payload) >> 8) & 0xFF)
    for b in payload: ck += b
    return hdr + payload + struct.pack('<H', ck & 0xFFFF)

ser = serial.Serial(port, BAUD, timeout=0.5)
time.sleep(0.5)
ser.reset_input_buffer()

# Send log class 3 and throttle
print('Sending LOG_CLASS=3 + THROTTLE=0.05')
ser.write(build_frame(0x03, struct.pack('<B', 3)))
ser.flush()
time.sleep(0.2)
ser.write(build_frame(0x01, struct.pack('<f', 0.05)))
ser.flush()

# Read raw bytes for 3 seconds
print('Reading raw bytes...')
all_bytes = b''
t0 = time.time()
while time.time() - t0 < 3.0:
    raw = ser.read(256)
    if raw:
        all_bytes += raw

print(f'Total bytes received: {len(all_bytes)}')
if all_bytes:
    print(f'First 200 bytes hex: {all_bytes[:200].hex()}')

# Look for 'db' sync patterns
pos = 0
frames_found = 0
while pos < len(all_bytes) - 1:
    if all_bytes[pos] == ord('d') and all_bytes[pos+1] == ord('b'):
        cmd = all_bytes[pos+2] if pos+2 < len(all_bytes) else 0
        size = 0
        if pos+5 < len(all_bytes):
            size = all_bytes[pos+4] | (all_bytes[pos+5] << 8)
        print(f'  Frame at offset {pos}: cmd=0x{cmd:02x} size={size}')
        frames_found += 1
        if frames_found > 20:
            break
    pos += 1

# Stop motor
ser.write(build_frame(0x01, struct.pack('<f', 0.0)))
ser.flush()
ser.close()
print('Done')
