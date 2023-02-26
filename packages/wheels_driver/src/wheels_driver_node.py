#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from wheels_driver.dagu_wheels_driver import DaguWheelsDriver
from std_srvs.srv import Trigger, TriggerResponse

from duckietown.dtros import DTROS, TopicType, NodeType

from abc import ABC, abstractmethod
from typing import Optional, Dict


class HWTest(ABC):
    @abstractmethod
    def test_id(self) -> str:
        """Short name, used to report start and end of test"""
        pass

    @abstractmethod
    def test_desc(self) -> str:
        """Test description and key params"""
        pass

    @abstractmethod
    def run_test(self) -> Optional[bool]:
        """return True or False if the result could be determined within the test"""
        pass


class HWTestMotor(HWTest):
    def __init__(self, driver, is_left: bool = True) -> None:
        super().__init__()
        self.vel = 0.5
        self.dura_secs = 5
        self._motor_driver = driver
        self._is_left = is_left

    def test_id(self) -> str:
        return f"Motor move ({'left' if self._is_left else 'right'})"

    def test_desc(self) -> str:
        return (
            f"The {'left' if self._is_left else 'right'} motor should start spinning."
            "\n"
            f"In about {self.dura_secs} seconds, it should stop moving."
        )

    def test_params(self) -> str:
        return f"[{self.test_id()}] vel = {self.vel}, dura_secs = {self.dura_secs}"

    def run_test(self) -> Optional[bool]:
        start_ts = rospy.Time.now()
        end_ts = start_ts + rospy.Duration(self.dura_secs)
        if self._is_left:
            self._motor_driver.set_wheels_speed(left=self.vel, right=0.0)
        else:
            self._motor_driver.set_wheels_speed(left=0.0, right=self.vel)
        while rospy.Time.now() < end_ts:
            rospy.sleep(1.0)
        self._motor_driver.set_wheels_speed(left=0.0, right=0.0)


class WheelsDriverNode(DTROS):
    """Node handling the motor velocities communication.

    Subscribes to the requested wheels commands (linear velocities, i.e. velocity for the left
    and the right wheels) and to an emergency stop flag.
    When the emergency flag `~emergency_stop` is set to `False` it actuates the wheel driver
    with the velocities received from `~wheels_cmd`. Publishes the execution of the commands
    to `~wheels_cmd_executed`.

    The emergency flag is `False` by default.

    Subscribers:
       ~wheels_cmd (:obj:`WheelsCmdStamped`): The requested wheel command
       ~emergency_stop (:obj:`BoolStamped`): Emergency stop. Can stop the actual execution of
           the wheel commands by the motors if set to `True`. Set to `False` for nominal
           operations.
    Publishers:
       ~wheels_cmd_executed (:obj:`WheelsCmdStamped`): Publishes the actual commands executed,
           i.e. when the emergency flag is `False` it publishes the requested command, and
           when it is `True`: zero values for both motors.

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(WheelsDriverNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)

        self.estop = False

        # Setup the driver
        self.driver = DaguWheelsDriver()

        # Initialize the executed commands message
        self.msg_wheels_cmd = WheelsCmdStamped()

        # Publisher for wheels command wih execution time
        self.pub_wheels_cmd = rospy.Publisher(
            "~wheels_cmd_executed", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.DRIVER
        )

        # Subscribers
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.wheels_cmd_cb, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.estop_cb, queue_size=1)

        # list of tests
        # (extra arg: is the target the left wheel)
        self._desc_tst1_srv = rospy.Service('~tests/move_left/desc', Trigger, lambda _: self._tst_desc(True))
        self._tst1_srv = rospy.Service('~tests/move_left/run', Trigger, lambda _: self._tst(True))
        self._desc_tst2_srv = rospy.Service('~tests/move_right/desc', Trigger, lambda _: self._tst_desc(False))
        self._tst2_srv = rospy.Service('~tests/move_right/run', Trigger, lambda _: self._tst(False))

        self.log("Initialized.")

    def _tst_desc(self, is_left):
        test = HWTestMotor(self.driver, is_left=is_left)
        return TriggerResponse(
            success=True,
            message=test.test_desc(),
        )

    def _tst(self, is_left):
        logs = []
        success = True
        try:
            test = HWTestMotor(self.driver, is_left=is_left)
            self.log(f"[{test.test_id()}] Started")
            self.log(test.test_params())
            logs.append(f"[{test.test_id()}] Started")
            logs.append(test.test_params())
            test.run_test()
            self.log(f"[{test.test_id()}] Finished")
            logs.append(f"[{test.test_id()}] Finished")
        except Exception as e:
            logs.append(f"Exception occured. Details: {e}")
            success = False

        return TriggerResponse(
            success=success,
            message="\n".join(logs),
        )

    def wheels_cmd_cb(self, msg):
        """
        Callback that sets wheels' speeds.

            Creates the wheels' speed message and publishes it. If the
            emergency stop flag is activated, publishes zero command.

            Args:
                msg (WheelsCmdStamped): velocity command
        """
        if self.estop:
            vel_left = 0.0
            vel_right = 0.0
        else:
            vel_left = msg.vel_left
            vel_right = msg.vel_right

        self.driver.set_wheels_speed(left=vel_left, right=vel_right)
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()
        self.msg_wheels_cmd.vel_left = vel_left
        self.msg_wheels_cmd.vel_right = vel_right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def estop_cb(self, msg):
        """
        Callback that enables/disables emergency stop

            Args:
                msg (BoolStamped): emergency_stop flag
        """

        self.estop = msg.data
        if self.estop:
            self.log("Emergency Stop Activated")
        else:
            self.log("Emergency Stop Released")

    def on_shutdown(self):
        """
        Shutdown procedure.

        Publishes a zero velocity command at shutdown.
        """
        self.driver.set_wheels_speed(left=0.0, right=0.0)


if __name__ == "__main__":
    # Initialize the node with rospy
    node = WheelsDriverNode(node_name="wheels_driver_node")
    # Keep it spinning to keep the node alive
    rospy.spin()
