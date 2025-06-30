#!/usr/bin/env python3

import math
from typing import Optional

import adafruit_mpu6050
import board
import rospy
import yaml
from adafruit_mpu6050 import MPU6050
from sensor_msgs.msg import Imu, Temperature
from std_srvs.srv import Empty

from hardware_test_imu import HardwareTestIMU
from duckietown.dtros import DTROS, NodeType

# TODO: calibration and loading custom config

class IMUNode(DTROS):
    def __init__(self):
        # Node Init
        super(IMUNode, self).__init__(node_name="imu_node", node_type=NodeType.DRIVER)

        # get ROS/Duckiebot parameters
        self._veh = rospy.get_param('~veh')
        self._i2c_connectors = rospy.get_param("~connectors", {})
        self._polling_hz = rospy.get_param("~polling_hz")
        self._temp_offset = rospy.get_param("~temp_offset")
        self._gyro_offset = rospy.get_param("~ang_vel_offset")
        self._accel_offset = rospy.get_param("~accel_offset")
        self.loginfo("===============IMU Node Init Val===============")
        self.loginfo(f"Op Rate: {self._polling_hz}")
        self.loginfo("Acceleration Offset: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % tuple(self._accel_offset))
        self.loginfo("Gyro Offset X:%.2f, Y: %.2f, Z: %.2f degrees/s" % tuple(self._gyro_offset))
        self.loginfo("Temp Offset: %.2f C" % self._temp_offset)
        self.loginfo("===============END of IMU Init Val===============")
        # IMU Initialization
        self._sensor: Optional[MPU6050] = self._find_sensor()
        if not self._sensor:
            conns: str = yaml.safe_dump(self._i2c_connectors, indent=2, sort_keys=True)
            self.logerr(f"No MPU6050 device found. These connectors were tested:\n{conns}\n")
            exit(1)
        # ---
        self.loginfo("===============Performing Initial Testing!===============")
        self.loginfo("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % self._sensor.acceleration)
        self.loginfo("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s" % self._sensor.gyro)
        self.loginfo("Temperature: %.2f C" % self._sensor.temperature)
        self.loginfo("===============IMU Initialization Complete===============")
        # ROS Pubsub initialization
        self.pub = rospy.Publisher('~data', Imu, queue_size=10)
        self.temp_pub = rospy.Publisher('~temperature', Temperature, queue_size=10)
        rospy.Service("~initialize_imu", Empty, self.zero_sensor)
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self._polling_hz), self.publish_data)
        
        # user hardware test
        self._hardware_test = HardwareTestIMU()

    def _find_sensor(self) -> Optional[MPU6050]:
        """
        Probe every (bus, address) pair listed in ~connectors.
        Accept both genuine MPU-6050 (WHO_AM_I = 0x68) and the newer
        ICM-2068x clones that return WHO_AM_I = 0x98.
        """
        # two IDs we know how to handle
        ALLOWED_IDS = (0x68, 0x71, 0x98)          # MPU-6050, ICM-2068x

        for c in self._i2c_connectors:
            bus_n = c["bus"]
            addr  = c["address"]

            self.loginfo(f"Trying IMU on I²C-{bus_n} @ 0x{addr:02X}")

            # ---------- read WHO_AM_I first ----------
            try:
                import smbus2
                with smbus2.SMBus(bus_n) as bus:
                    who = bus.read_byte_data(addr, 0x75)
            except FileNotFoundError:
                self.logwarn(f"I²C bus {bus_n} does not exist")
                continue
            except OSError as e:
                self.logwarn(f"No response at 0x{addr:02X} on bus {bus_n}: {e}")
                continue

            if who not in ALLOWED_IDS:
                self.logwarn(
                    f"Unknown IMU (WHO_AM_I 0x{who:02X}) at 0x{addr:02X} on bus {bus_n}"
                )
                continue

            # ---------- patch Adafruit driver and instantiate ----------
            adafruit_mpu6050._MPU6050_DEVICE_ID = who       # make the sanity-check happy

            try:
                sensor = adafruit_mpu6050.MPU6050(board.I2C(), address=addr)
                self.loginfo(
                    f"Found IMU (ID 0x{who:02X}) on I²C-{bus_n} @ 0x{addr:02X}"
                )
                return sensor
            except RuntimeError as e:
                self.logwarn(f"Driver rejected device at 0x{addr:02X}: {e}")
                continue

        # nothing worked
        return None

    def publish_data(self, event):
        # Message Blank
        msg = Imu()
        temp_msg = Temperature()
        # Poll Sensor
        try:
            # You take the time immediately when you are polling imu
            msg.header.stamp = temp_msg.header.stamp = rospy.Time.now()
            acc_data = self._sensor.acceleration
            gyro_data = self._sensor.gyro
            temp_data = self._sensor.temperature
            # Do it together so that the timestamp is honored
            # Populate Message
            msg.header.frame_id = temp_msg.header.frame_id = f"{self._veh}/imu"
            # Orientation (we do not have this data)
            msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0
            # If you have no estimate for one of the data elements
            # set element 0 of the associated covariance matrix to -1
            msg.orientation_covariance = [0.0 for _ in range(len(msg.orientation_covariance))]
            msg.orientation_covariance[0] = -1
            # Angular Velocity
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = tuple(
                gyro_data[i] - self._gyro_offset[i] for i in range(len(gyro_data))
            )
            msg.angular_velocity_covariance = [0.0 for _ in range(len(msg.angular_velocity_covariance))]
            # Acceleration
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = tuple(
                (acc_data[i] - self._accel_offset[i]) for i in range(len(acc_data)))
            msg.linear_acceleration_covariance = [0.0 for _ in range(len(msg.linear_acceleration_covariance))]
            # Pub
            self.pub.publish(msg)
            temp_msg.temperature = temp_data - self._temp_offset
            self.temp_pub.publish(temp_msg)

        except Exception as IMUCommLoss:
            self.logwarn(f"IMU Comm Loss: {IMUCommLoss}")
            pass
        return

    def zero_sensor(self, _):
        acc_data = self._sensor.acceleration
        gyro_data = self._sensor.gyro
        temp_data = self._sensor.temperature
        self._gyro_offset = list(gyro_data)
        self._accel_offset = list(acc_data)
        self._temp_offset = temp_data
        self.loginfo("IMU zeroed with ACC: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % acc_data)
        self.loginfo("IMU zeroed with Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s" % gyro_data)
        return []


if __name__ == '__main__':
    node = IMUNode()
    rospy.spin()
