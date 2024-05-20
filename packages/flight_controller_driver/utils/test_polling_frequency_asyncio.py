import time
import asyncio
from yamspy import MSPy

# Define the function to send RC commands
async def send_rc_commands(msp):
    rc_channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
    start_time = time.perf_counter()
    count = 0
    while time.perf_counter() - start_time < 10:
        if msp.send_RAW_RC(rc_channels):
            dataHandler = msp.receive_msg()
            msp.process_recv_data(dataHandler)
        count += 1
        await asyncio.sleep(0)  # Yield control to the event loop
    return count / 10  # Return the frequency

# Define the function to read IMU data
async def read_imu(msp):
    start_time = time.perf_counter()
    count = 0
    while time.perf_counter() - start_time < 10:
        msp.fast_read_imu()
        count += 1
        await asyncio.sleep(0)  # Yield control to the event loop
    return count / 10  # Return the frequency

# Define the function to read attitude data
async def read_attitude(msp : MSPy):
    start_time = time.perf_counter()
    count = 0
    while time.perf_counter() - start_time < 10:
        attitude_data = msp.fast_read_attitude()
        if attitude_data:
            count += 1
        await asyncio.sleep(0)  # Yield control to the event loop
    return count / 10  # Return the frequency

async def main():
    # Initialize MSPy object
    msp = MSPy(device="/dev/ttyACM0", loglevel="DEBUG")

    with MSPy(device="/dev/ttyACM0", loglevel="DEBUG") as msp:
        # Run tasks concurrently
        rc_task = asyncio.create_task(send_rc_commands(msp))
        imu_task = asyncio.create_task(read_imu(msp))
        attitude_task = asyncio.create_task(read_attitude(msp))

        # Wait for all tasks to complete
        await asyncio.gather(rc_task, imu_task, attitude_task)

        # Calculate frequencies
        rc_frequency = await rc_task
        imu_frequency = await imu_task
        attitude_frequency = await attitude_task

        # Print the results
        print(f"RC Command Frequency: {rc_frequency} Hz")
        print(f"IMU Read Frequency: {imu_frequency} Hz")
        print(f"Attitude Read Frequency: {attitude_frequency} Hz")


if __name__ == "__main__":
    asyncio.run(main())
