#!/usr/bin/env python3

import math
from typing import Optional

import adafruit_mpu6050
import board
import copy
import numpy as np
import rospy
import yaml
from adafruit_mpu6050 import MPU6050
from sensor_msgs.msg import Imu, Temperature
from std_srvs.srv import Empty
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from scipy.spatial.transform import Rotation as R

from hardware_test_imu import HardwareTestIMU
from duckietown.dtros import DTROS, NodeType

# TODO: calibration and loading custom config

# Default publishing frame: the IMU data would make sense to humans when published in this frame, regardless where it's mounted
DEFAULT_IMU_PUB_FRAME = "{robot_hostname}/bottom_plate"
# when transform cannot be found, use the plain imu frame
FALLBACK_IMU_PUB_FRAME = "{robot_hostname}/imu"
# Max timeout (secs), for TF between above frames to become available
TF_INIT_TIMEOUT_SECS = 10


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

        # start look for TFs
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer)
        # human sensible frame (default) and sensor frame (fallback)
        self._default_pub_frame = DEFAULT_IMU_PUB_FRAME.format(robot_hostname=self._veh)
        self._fallback_pub_frame = FALLBACK_IMU_PUB_FRAME.format(robot_hostname=self._veh)
        if self._tf_buffer.can_transform(
            target_frame=self._default_pub_frame,
            source_frame=self._fallback_pub_frame,
            time=rospy.Time(0),  # latest
            timeout=rospy.Duration.from_sec(TF_INIT_TIMEOUT_SECS),
        ):
            self._imu_pub_frame = self._default_pub_frame
            _static_transform = self._tf_buffer.lookup_transform(self._fallback_pub_frame, self._default_pub_frame, rospy.Time(0))
            self._frame_rotation = R.from_quat([
                _static_transform.transform.rotation.x,
                _static_transform.transform.rotation.y,
                _static_transform.transform.rotation.z,
                _static_transform.transform.rotation.w,
            ])
            self.loginfo("[IMU TF Lookup] Success!")
        else:
            # failed to obtain default->fallback tf link, use fallback frame
            self._imu_pub_frame = self._fallback_pub_frame
            self._frame_rotation = None
            self.logwarn("[IMU TF Lookup] Failed!")
        self.loginfo(f"Publishing IMU data in frame: {self._imu_pub_frame}")

        # ROS Pubsub initialization
        self.pub = rospy.Publisher('~data', Imu, queue_size=10)
        self.temp_pub = rospy.Publisher('~temperature', Temperature, queue_size=10)
        rospy.Service("~initialize_imu", Empty, self.zero_sensor)
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self._polling_hz), self.publish_data)
        
        # user hardware test
        self._hardware_test = HardwareTestIMU()

    def _find_sensor(self) -> Optional[MPU6050]:
        for connector in self._i2c_connectors:
            conn: str = "[bus:{bus}](0x{address:02X})".format(**connector)
            self.loginfo(f"Trying to open device on connector {conn}")
            # Overwrite Adafruit default device ID
            adafruit_mpu6050._MPU6050_DEVICE_ID = connector["address"]
            try:
                sensor = MPU6050(board.I2C())
            except Exception:
                self.logwarn(f"No devices found on connector {conn}, but the bus exists")
                continue
            self.loginfo(f"Device found on connector {conn}")
            return sensor

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

            # perform transfrom if applicable
            if self._imu_pub_frame != self._fallback_pub_frame:
                ang_vel = [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ]
                linear_acc = [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ]

                # for IMU, only apply rotation among frames
                ang_vel = self._frame_rotation.apply(ang_vel)
                linear_acc = self._frame_rotation.apply(linear_acc)

                # store transformed values to ros msg
                msg.angular_velocity.x = ang_vel[0]
                msg.angular_velocity.y = ang_vel[1]
                msg.angular_velocity.z = ang_vel[2]
                msg.linear_acceleration.x = linear_acc[0]
                msg.linear_acceleration.y = linear_acc[1]
                msg.linear_acceleration.z = linear_acc[2]

            # set same frame for temperature message to be consistent 
            msg.header.frame_id = temp_msg.header.frame_id = self._imu_pub_frame

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
