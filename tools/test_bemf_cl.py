#!/usr/bin/env python3
"""Test BEMF closed-loop transition — log class 8 (BEMF observer data)
Logs: [0]state [1]bemf_theta [2]ol_theta [3]enc_elec
      [4]flux_a [5]flux_b [6]bemf_mag [7]bemf_speed
"""
import serial, struct, time, sys

BAUD = 19200
port = '/dev/cu.usbmodem31203'
THROTTLE = 0.3
DURATION = 8.0   # seconds of data capture
LOG_CLASS = 8    # BEMF observer log

def build_frame(cmd, payload):
    hdr = struct.pack('<2sBBH', b'db', cmd, 0, len(payload))
    ck = cmd + 0 + (len(payload) & 0xFF) + ((len(payload) >> 8) & 0xFF)
    for b in payload:
        ck += b
    return hdr + payload + struct.pack('<H', ck & 0xFFFF)

ser = serial.Serial(port, BAUD, timeout=0.1)
time.sleep(0.5)
ser.reset_input_buffer()

# Stop motor, set log class
ser.write(build_frame(0x01, struct.pack('<f', 0.0)))
ser.flush()
time.sleep(0.5)

ser.write(build_frame(0x03, bytes([LOG_CLASS])))
ser.flush()
time.sleep(0.1)

# Start motor
ser.write(build_frame(0x01, struct.pack('<f', THROTTLE)))
ser.flush()
print(f"Motor started at throttle={THROTTLE}, log class={LOG_CLASS}")

t0 = time.time()
buf = bytearray()
samples = 0
states_seen = set()
last_state = -1

while time.time() - t0 < DURATION:
    data = ser.read(200)
    if data:
        buf.extend(data)
        while len(buf) >= 40:
            if buf[0] == ord('d') and buf[1] == ord('b'):
                cmd = buf[2]
                sz = buf[4] | (buf[5] << 8)
                if cmd == 0x00 and sz == 32:
                    payload = buf[6:6+32]
                    f = struct.unpack('<8f', payload)
                    state = int(f[0])
                    bemf_theta = f[1]
                    ol_theta = f[2]
                    enc_elec = f[3]
                    flux_a = f[4]
                    flux_b = f[5]
                    bemf_mag = f[6]
                    bemf_speed = f[7]

                    states_seen.add(state)
                    if state != last_state:
                        print(f"\n>>> STATE CHANGE: {last_state} -> {state}")
                        last_state = state

                    angle_diff = bemf_theta - ol_theta
                    if angle_diff > 3.14159:
                        angle_diff -= 6.28318
                    if angle_diff < -3.14159:
                        angle_diff += 6.28318

                    label = ['IDLE','ALIGN','RAMP','OL','BLEND','CL'][state] if state < 6 else '?'
                    print(f"[{label:5s}] bemf={bemf_theta:5.2f} ol={ol_theta:5.2f} "
                          f"diff={angle_diff:+6.2f} enc={enc_elec:5.2f} "
                          f"mag={bemf_mag:.4f} spd={bemf_speed:+8.1f} "
                          f"flux=({flux_a:+.4f},{flux_b:+.4f})")
                    samples += 1
                    buf = buf[40:]
                    continue
            buf.pop(0)

# Stop motor
ser.write(build_frame(0x01, struct.pack('<f', 0.0)))
ser.flush()
time.sleep(0.2)
ser.close()

print(f"\nDone. {samples} samples, states seen: {sorted(states_seen)}")
if 5 in states_seen:
    print("SUCCESS: Reached CLOSEDLOOP state (5)")
else:
    print("WARNING: Never reached CLOSEDLOOP state")
