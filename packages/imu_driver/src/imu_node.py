#!/usr/bin/env python3
import math

import adafruit_mpu6050  # IMU Driver
import board
import rospy
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from sensor_msgs.msg import Imu


# TODO: calibration and loading custom config

class IMUNode(DTROS):
    def __init__(self):
        # Node Init
        super(IMUNode, self).__init__(node_name="imu_node", node_type=NodeType.DRIVER)

        # get ROS/Duckiebot parameters
        self._imu_device_id = rospy.get_param('~imu_device_id', 0x71)
        print("IMU ID: " + self._imu_device_id)
        adafruit_mpu6050._MPU6050_DEVICE_ID = 0x71  # Overwrite Adafruit default device ID being wrong
        self._veh = rospy.get_param('~veh')
        polling_hz = rospy.get_param("~polling_hz")
        self._ang_vel_offset = DTParam("~ang_vel_offset", param_type=ParamType.LIST, help="Angular velocity offsets")
        self._accel_offset = DTParam("~accel_offset", param_type=ParamType.LIST, help="Acceleration offset")

        # IMU Initialization
        try:
            self._imu = adafruit_mpu6050.MPU6050(board.I2C())
            self.loginfo("===============Performing Initial Testing!===============")
            self.loginfo("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % self._imu.acceleration)
            self.loginfo("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s" % self._imu.gyro)
            self.loginfo("Temperature: %.2f C" % self._imu.temperature)
            self.loginfo("===============IMU Initialization Complete===============")
        except Exception as IMUInitException:
            self.logerr("IMU sensor not correctly setup! Error:")
            self.logerr(IMUInitException)
            exit(1)

        # ROS Pubsub initialization
        self.pub = rospy.Publisher('~imu_data', Imu, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / polling_hz), self.publish_data)

    def publish_data(self, event):
        # Message Blank
        msg = Imu()
        # Poll Sensor
        try:
            # You take the time immediately when you are polling imu
            msg.header.stamp = rospy.Time.now()
            acc_data = self._imu.acceleration
            gyro_data = self._imu.gyro
            temp_data = self._imu.temperature
            # Do it together so that the timestamp is honored
            self.logdebug("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % acc_data)
            self.logdebug("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s" % gyro_data)
            self.logdebug("Temperature: %.2f C" % temp_data)
            # Populate Message
            msg.header.frame_id = f"{self._veh}/imu/bottom_center"  # TODO
            # Orientation
            msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0  # We do not have this data
            # If you have no estimate for one of the data elements
            # set element 0 of the associated covariance matrix to -1
            msg.orientation_covariance = [0.0 for _ in range(len(msg.orientation_covariance))]
            msg.orientation_covariance[0] = -1
            # Angular Velocity
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = tuple(
                math.pi / 180.0 * x for x in gyro_data)
            msg.angular_velocity_covariance = [0.0 for _ in range(len(msg.angular_velocity_covariance))]
            # Acceleration
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = tuple(x for x in acc_data)
            msg.linear_acceleration_covariance = [0.0 for _ in range(len(msg.linear_acceleration_covariance))]
            # Pub
            self.pub.publish(msg)
        except Exception as IMUCommLoss:
            self.logwarn(f"IMU Comm Loss: {IMUCommLoss}")
            pass
        return


if __name__ == '__main__':
    node = IMUNode()
    rospy.spin()
