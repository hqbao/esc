import serial, struct, time

BAUD = 19200 
port = '/dev/cu.usbmodem31203'

def build_frame(cmd, payload):
    hdr = struct.pack('<2sBBH', b'db', cmd, 0, len(payload))
    ck = cmd + 0 + (len(payload) & 0xFF) + ((len(payload) >> 8) & 0xFF)
    for b in payload: ck += b
    return hdr + payload + struct.pack('<H', ck & 0xFFFF)

ser = serial.Serial(port, BAUD, timeout=0.1)
time.sleep(0.5)
ser.reset_input_buffer()

ser.write(build_frame(0x01, struct.pack('<f', 0.0)))
ser.flush()
time.sleep(0.5)

ser.write(build_frame(0x03, bytes([5])))
ser.flush()
time.sleep(0.1)

ser.write(build_frame(0x01, struct.pack('<f', 0.3)))
ser.flush()

print('Reading...')
t0 = time.time()
buf = bytearray()
while time.time() - t0 < 6.0:
    data = ser.read(100)
    if data:
        buf.extend(data)
        while len(buf) >= 40:
            if buf[0] == ord('d') and buf[1] == ord('b'):
                cmd = buf[2]
                sz = buf[4] | (buf[5] << 8)
                if cmd == 0x00 and sz == 32:
                    payload = buf[6 : 6+32]
                    f = struct.unpack('<8f', payload)
                    print(f"ST:{f[0]:.0f} Id:{f[1]:.2f} Iq:{f[2]:.2f} e_e:{f[3]:.2f} Vq:{f[4]:.2f} cl_Vq:{f[5]:.2f}")
                    buf = buf[40:]
                    continue
            buf.pop(0)

ser.write(build_frame(0x01, struct.pack('<f', 0.0)))
ser.flush()
ser.close()
print('Done')
