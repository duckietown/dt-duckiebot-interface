import time
from yamspy import MSPy
import threading

BAUDRATE = 5e5
DEV = "/dev/ttyACM0"

# Define the function to send RC commands
def send_rc_commands(msp : MSPy):
    rc_channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
    start_time = time.perf_counter()
    count = 0
    while time.perf_counter() - start_time < 10:
        if msp.send_RAW_RC(rc_channels):
            dataHandler = msp.receive_msg()
            result = msp.process_recv_data(dataHandler)
            
            count += 1
        msp.fast_read_imu()
        # attitude_data = msp.fast_read_attitude()
    return count / 10  # Return the frequency

# Define the function to read IMU data
def read_imu(msp : MSPy):
    start_time = time.perf_counter()
    count = 0
    while time.perf_counter() - start_time < 10:
        msp.fast_read_imu()
        count += 1
    return count / 10  # Return the frequency

# Define the function to read attitude data
def read_attitude(msp : MSPy):
    start_time = time.perf_counter()
    count = 0
    while time.perf_counter() - start_time < 10:
        attitude_data = msp.fast_read_attitude()
        if attitude_data:
            count += 1
    return count / 10  # Return the frequency

def main():
    # Initialize MSPy object
    with MSPy(device=DEV, baudrate = BAUDRATE, loglevel="DEBUG") as msp:
        time.sleep(1)
        # Calculate frequencies
        rc_frequency = send_rc_commands(msp)

        # Print the results
        print(f"RC Command Frequency: {rc_frequency} Hz")

    # Close the connection

if __name__ == "__main__":
    main()
