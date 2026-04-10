import serial, struct, time

port = '/dev/cu.usbmodem31203'
baud = 19200
ser = serial.Serial(port, baud, timeout=1)

def send_topic(topic_id, data):
    payload_len = len(data)
    frame = bytearray([0x55, 0x42, topic_id, payload_len]) + data
    chk_a, chk_b = 0, 0
    for b in frame[2:]:
        chk_a = (chk_a + b) & 0xFF
        chk_b = (chk_b + chk_a) & 0xFF
    frame.extend([chk_a, chk_b])
    ser.write(frame)

def set_throttle(throttle):
    send_topic(10, struct.pack('<f', throttle))  # Guessing topic for motor_throttle? Let's check messages.h. Or wait, I don't know the topic IDs unless I look.
