#!/usr/bin/env python3
import math

import adafruit_mpu6050  # IMU Driver
import board
import rospy
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from sensor_msgs.msg import Imu, Temperature
from std_srvs.srv import Empty

from abc import ABC, abstractmethod
from typing import Optional, Dict
from std_srvs.srv import Trigger, TriggerResponse


class HWTest(ABC):
    @abstractmethod
    def test_id(self) -> str:
        """Short name, used to report start and end of test"""
        pass

    @abstractmethod
    def test_desc_preparation(self) -> str:
        """Preparation before running. E.g. put the DB upside down"""
        pass

    def test_desc_running(self) -> str:
        """Actual steps to run the test"""
        # default: just click the "Run test" button
        return "Run the test"

    @abstractmethod
    def test_desc_expectation(self) -> str:
        """Expected outcome"""
        pass

    @abstractmethod
    def test_desc_log_gather(self) -> str:
        """How to gather logs before reporting"""
        pass

    def test_desc(self) -> str:
        """Test description and key params"""
        # TODO: use JSON and keys to separate sections
        return "\n\n".join([
            self.test_desc_preparation(),
            self.test_desc_expectation(),
            self.test_desc_running(),
            self.test_desc_log_gather(),
        ])

    @abstractmethod
    def run_test(self) -> Optional[bool]:  # TODO: decide whether auto grade or not
        """return True or False if the result could be determined within the test"""
        pass


class HWTestIMU(HWTest):
    def __init__(self) -> None:
        super().__init__()
        self._desc_tst_srv = rospy.Service('~tests/desc', Trigger, self._tst_desc)
        self._tst_srv = rospy.Service('~tests/run', Trigger, self._tst)
        # test settings
        self.dura_secs = 3
        self.abs_min_incline_d_accel = 5.0

    def _tst_desc(self, _):
        return TriggerResponse(
            success=True,
            message=self.test_desc(),
        )

    def _tst(self, _):
        msg_start = None
        msg_end = None

        # Subscribe to the topic and get one message
        try:
            # initial measurement
            msg_start = rospy.wait_for_message('~imu_data', Imu, timeout=1.0)
            success = True
        except rospy.ROSException as e:
            success = False

        rospy.sleep(self.dura_secs)

        if success:
            try:
                # final measurement
                msg_end = rospy.wait_for_message('~imu_data', Imu, timeout=1.0)
                success = True
            except rospy.ROSException as e:
                success = False

        if success:
            dx = msg_start.linear_acceleration.x - msg_end.linear_acceleration.x
            response = "No inclination along x axis detected."
            if abs(dx) > self.abs_min_incline_d_accel:
                if dx < 0:
                    response = "Inclined to the left (<-)"
                else:
                    response = "Inclined to the right (->)"
        else:
            response = "Failed to take IMU measurements"

        # Return the service response
        return TriggerResponse(success=success, message=response)

    def test_id(self) -> str:
        return f"IMU"

    def test_desc_preparation(self) -> str:
        return "Put your Duckiebot in normal orientation, on a surface (e.g. desk) which allows tilting it."

    def test_desc_running(self) -> str:
        return (
            "Hold your Duckiebot\n"
            "Once the test starts, you will see a flat plane reflecting your Duckiebot.\n"
            "Your objective is to keep ball on the plane by tilting the robot.\n"
            "Now, click the button below to run the test"
        )

    def test_desc_expectation(self) -> str:
        return (
            "If the plane does not move like your Duckiebot, there is a problem.\n"
            "The test is considered successful, if you're able to control the plane naturally with moving your Duckiebot."
        )

    # # Instructions for preliminary test: i.e. only tilting and let program tell the direction
    # def test_desc_running(self) -> str:
    #     return (
    #         "Put your hand near the Duckiebot\n"
    #         "Once the test starts, you should tilt the robot left/right, i.e. make the top display face your left/right. And the robot will be standing on the side of either left or right wheel.\n"
    #         "Now, click the button below to run the test"
    #     )

    # def test_desc_expectation(self) -> str:
    #     return (
    #         f"The experiment will finish in {self.dura_secs} seconds.\n"
    #         "After finishing, the response string below should indicate, whether the robot is inclined to the left or the right.\n"
    #         "You should run the test MULTIPLE times (left, standing still, right), to verify."
    #     )
    
    def test_desc_log_gather(self) -> str:
        return (
            "On your laptop, run the following command to save the logs.\n"
            "Replace the `[path/to/save]' to the directory path where you would like to save the logs.\n"
            "`docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt'\n"
            "Also, right click in the web browser and choose the `Inspect' option.\n"
            "Then, navigate to the `Console' tab and copy any error messages."
        )

    def test_params(self) -> str:
        return f"[{self.test_id()}] dura_secs = {self.dura}, abs_min_incline_d_accel = {self.abs_min_incline_d_accel}"

    def run_test(self) -> Optional[bool]:
        pass

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

        self.hw_test = HWTestIMU()

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
