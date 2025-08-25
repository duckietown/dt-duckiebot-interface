from dt_robot_utils import get_robot_hardware, RobotHardware

if get_robot_hardware() != RobotHardware.VIRTUAL:
    from imu_driver.imu_driver import IMUDriverCalibratedMPU6050 as IMUDriver
else:
    from imu_driver.virtual_imu_driver import VirtualIMUDriver as IMUDriver
