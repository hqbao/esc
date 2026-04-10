import serial, struct, time

port = '/dev/cu.usbmodem31203'
try:
    ser = serial.Serial(port, 115200, timeout=0.1) # Actually 115200 or 19200? Wait, DBLink tool uses 115200 usually, but let me check. Actually DBLink in this project is 19200 maybe? Let's check debug_serial.py
except Exception as e:
    print(e)
    exit(1)
