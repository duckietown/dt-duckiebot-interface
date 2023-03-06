#!/usr/bin/env python3
import math

import adafruit_mpu6050  # IMU Driver
import board
import rospy
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from sensor_msgs.msg import Imu, Temperature
from std_srvs.srv import Empty


# TODO: calibration and loading custom config

class IMUNode(DTROS):
    def __init__(self):
        # Node Init
        super(IMUNode, self).__init__(node_name="imu_node", node_type=NodeType.DRIVER)

        # get ROS/Duckiebot parameters
        self._imu_device_id = rospy.get_param('~imu_device_id', 0x71)
        adafruit_mpu6050._MPU6050_DEVICE_ID = 0x71  # Overwrite Adafruit default device ID being wrong
        self._veh = rospy.get_param('~veh')
        polling_hz = rospy.get_param("~polling_hz")
        self._temp_offset = rospy.get_param("~temp_offset")
        self._gyro_offset = rospy.get_param("~ang_vel_offset")
        self._accel_offset = rospy.get_param("~accel_offset")
        self.loginfo("===============IMU Node Init Val===============")
        self.loginfo(f"Op Rate: {polling_hz}")
        self.loginfo(f"ADDR: {self._imu_device_id}")
        self.loginfo("Acceleration Offset: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % tuple(self._accel_offset))
        self.loginfo("Gyro Offset X:%.2f, Y: %.2f, Z: %.2f degrees/s" % tuple(self._gyro_offset))
        self.loginfo("Temp Offset: %.2f C" % self._temp_offset)
        self.loginfo("===============END of IMU Init Val===============")
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
        self.temp_pub = rospy.Publisher('~temp_data', Temperature, queue_size=10)
        rospy.Service("~initialize_imu", Empty, self.zero_sensor)
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / polling_hz), self.publish_data)

    def publish_data(self, event):
        # Message Blank
        msg = Imu()
        temp_msg = Temperature()
        # Poll Sensor
        try:
            # You take the time immediately when you are polling imu
            msg.header.stamp = temp_msg.header.stamp = rospy.Time.now()
            acc_data = self._imu.acceleration
            gyro_data = self._imu.gyro
            temp_data = self._imu.temperature
            # Do it together so that the timestamp is honored

            # Populate Message
            msg.header.frame_id = temp_msg.header.frame_id = f"{self._veh}/imu"

            # Orientation
            msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0  # We do not have this data
            # If you have no estimate for one of the data elements
            # set element 0 of the associated covariance matrix to -1
            msg.orientation_covariance = [0.0 for _ in range(len(msg.orientation_covariance))]
            msg.orientation_covariance[0] = -1
            # Angular Velocity
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = tuple(
                math.pi / 180.0 * (gyro_data[i] - self._gyro_offset[i]) for i in range(len(gyro_data)))
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

    def zero_sensor(self, req):
        acc_data = self._imu.acceleration
        gyro_data = self._imu.gyro
        temp_data = self._imu.temperature
        self._gyro_offset = list(gyro_data)
        self._accel_offset = list(acc_data)
        self._temp_offset = temp_data
        self.loginfo("IMU zeroed with ACC: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % acc_data)
        self.loginfo("IMU zeroed with Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s" % gyro_data)
        return []


if __name__ == '__main__':
    node = IMUNode()
    rospy.spin()
