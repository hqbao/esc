#!/usr/bin/env python3
"""Multi-throttle BEMF CL test — tests at 0.1, 0.3, 0.6 throttle
Uses log class 3 (VOLTAGE): [0]state [1]Id [2]Iq [3]theta [4]bemf_theta [5]enc_elec [6]Ia [7]Ib
"""
import serial, struct, time

BAUD = 19200
port = '/dev/cu.usbmodem31203'
THROTTLES = [0.1, 0.3, 0.6]
RUN_TIME = 5.0   # seconds per throttle level
LOG_CLASS = 3    # VOLTAGE log

def build_frame(cmd, payload):
    hdr = struct.pack('<2sBBH', b'db', cmd, 0, len(payload))
    ck = cmd + 0 + (len(payload) & 0xFF) + ((len(payload) >> 8) & 0xFF)
    for b in payload:
        ck += b
    return hdr + payload + struct.pack('<H', ck & 0xFFFF)

def parse_frames(buf, results):
    consumed = 0
    while len(buf) >= 40:
        if buf[0] == ord('d') and buf[1] == ord('b'):
            cmd = buf[2]
            sz = buf[4] | (buf[5] << 8)
            if cmd == 0x00 and sz == 32:
                payload = buf[6:6+32]
                f = struct.unpack('<8f', payload)
                results.append(f)
                buf = buf[40:]
                consumed += 40
                continue
        buf.pop(0)
        consumed += 1
    return consumed

ser = serial.Serial(port, BAUD, timeout=0.1)
time.sleep(0.5)
ser.reset_input_buffer()

# Ensure stopped
ser.write(build_frame(0x01, struct.pack('<f', 0.0)))
ser.flush()
time.sleep(0.5)

# Set log class
ser.write(build_frame(0x03, bytes([LOG_CLASS])))
ser.flush()
time.sleep(0.1)

all_pass = True

for throttle in THROTTLES:
    print(f"\n{'='*60}")
    print(f"Testing throttle = {throttle}")
    print(f"{'='*60}")
    
    ser.reset_input_buffer()
    ser.write(build_frame(0x01, struct.pack('<f', throttle)))
    ser.flush()
    
    t0 = time.time()
    buf = bytearray()
    results = []
    
    while time.time() - t0 < RUN_TIME:
        data = ser.read(200)
        if data:
            buf.extend(data)
            parse_frames(buf, results)
    
    # Stop motor
    ser.write(build_frame(0x01, struct.pack('<f', 0.0)))
    ser.flush()
    time.sleep(1.0)
    
    # Analyze results
    states = [int(r[0]) for r in results]
    cl_samples = [r for r in results if int(r[0]) == 5]  # CL state
    blend_samples = [r for r in results if int(r[0]) == 4]
    
    states_seen = sorted(set(states))
    print(f"Samples: {len(results)}, States seen: {states_seen}")
    
    if 5 in states_seen:
        print(f"  ✓ CLOSEDLOOP reached ({len(cl_samples)} CL samples)")
        
        if cl_samples:
            ids = [s[1] for s in cl_samples]
            iqs = [s[2] for s in cl_samples]
            thetas = [s[3] for s in cl_samples]
            
            avg_id = sum(ids) / len(ids)
            avg_iq = sum(iqs) / len(iqs)
            max_id = max(abs(x) for x in ids)
            max_iq = max(abs(x) for x in iqs)
            
            print(f"  Id: avg={avg_id:+.3f}  max|Id|={max_id:.3f}")
            print(f"  Iq: avg={avg_iq:+.3f}  max|Iq|={max_iq:.3f}")
            
            # Show a few CL samples
            for s in cl_samples[:3]:
                print(f"    ST:{s[0]:.0f} Id:{s[1]:+.3f} Iq:{s[2]:+.3f} θ:{s[3]:.2f} bemf_θ:{s[4]:.2f}")
    else:
        print(f"  ✗ FAILED — never reached CLOSEDLOOP")
        all_pass = False
    
    if 4 in states_seen and blend_samples:
        print(f"  Blend samples: {len(blend_samples)}")

ser.close()

print(f"\n{'='*60}")
if all_pass:
    print("ALL THROTTLES PASSED — Motor reached CL at all levels")
else:
    print("SOME THROTTLES FAILED")
print(f"{'='*60}")
