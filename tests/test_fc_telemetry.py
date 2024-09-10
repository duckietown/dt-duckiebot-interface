import asyncio
import time
from mavsdk import System

async def main(port = "/dev/ttyACM0", baudrate = 921600):
    board = System()
    await board.connect(f"serial://{port}:{baudrate}")

    await board.telemetry.set_rate_imu(rate_hz=50)
    # Start the IMU callback
    asyncio.ensure_future(imu_callback(board))
    asyncio.ensure_future(print_attitude(board))
    asyncio.ensure_future(print_battery(board))
    while True:
        await asyncio.sleep(1)

async def print_battery(board : System):
    count = 0
    start_time = time.time()
    duration = 10  # Duration to measure the rate, in seconds

    async for battery in board.telemetry.battery():
        count += 1
        current_time = time.time()
        
        # Check if the duration has passed
        if current_time - start_time >= duration:
            rate = count / duration
            print(f"Battery rate: {rate:.2f} messages per second")
            # Reset for the next duration measurement
            count = 0
            start_time = current_time

async def imu_callback(board : System):
    count = 0
    start_time = time.time()
    duration = 10  # Duration to measure the rate, in seconds
         
    async for imu in board.telemetry.imu():
        count += 1
        current_time = time.time()
        
        # Check if the duration has passed
        if current_time - start_time >= duration:
            rate = count / duration
            # print(f"Acceleration data: {imu.acceleration_frd.x:.2f}, {imu.acceleration_frd.y:.2f}, {imu.acceleration_frd.z:.2f}")
            # print(f"Angular velocity data: {imu.angular_velocity_frd.x:.2f}, {imu.angular_velocity_frd.y:.2f}, {imu.angular_velocity_frd.z:.2f}")
            print(f"IMU rate: {rate:.2f} messages per second")
            # Reset for the next duration measurement
            count = 0
            start_time = current_time

async def print_attitude(drone : System):
    count = 0
    start_time = time.time()
    duration = 10  # Duration to measure the rate, in seconds
    
    async for attitude in drone.telemetry.attitude_euler():
        count += 1
        current_time = time.time()
        
        # Check if the duration has passed
        if current_time - start_time >= duration:
            rate = count / duration
            print(f"Attitude rate: {rate:.2f} messages per second")
            # print(f"Roll: {attitude.roll_deg:.2f}, Pitch: {attitude.pitch_deg:.2f}, Yaw: {attitude.yaw_deg:.2f}")
            # Reset for the next duration measurement
            count = 0
            start_time = current_time

async def azip(*async_iterables):
    iterators = [aiter(it) for it in async_iterables]
    
    while True:
        try:
            results = await asyncio.gather(*(anext(it) for it in iterators))
            yield tuple(results)
        except StopAsyncIteration:
            return
                  
if __name__ == "__main__":
    asyncio.run(main())