import serial, sys, time

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM24"
BAUD = 115200
time.sleep(2)

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    for _ in range(30):
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line:
            print(line)
    ser.close()
except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
