import serial
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 5e5

def test_serial_port_speed():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    start_time = time.perf_counter()
    count = 0
    test_data = b'x' * 1024  # 1 KB of test data
    while time.perf_counter() - start_time < 10:
        ser.write(test_data)
        ser.read(1024)  # Read back the same amount of data
        count += 1
    ser.close()
    return count * 1024 / 10  # Bytes per second

speed = test_serial_port_speed()
print(f"Serial Port Speed: {speed / 1e3} KB/s")
