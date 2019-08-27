#!/usr/bin/env python
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from wheels_driver.dagu_wheels_driver import DaguWheelsDriver


class WheelsDriverNode(DTROS):
    """Node handling the motor velocities communication.

        Subscribes to the requested wheels commands (linear velocities) and
        to an emergency stop flag. Publishes the execution of the commands

        Subscribers:
            sub_topic:
                topic: ~wheels_cmd
                type: WheelsCmdStamped
            sub_e_stop:
                topic: ~emergency_stop
                type: BoolStamped
        Publishers:
            pub_wheels_cmd:
                topic: ~wheels_cmd_executed
                type: WheelsCmdStamped
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(WheelsDriverNode, self).__init__(node_name=node_name)

        self.estop = False

        # Setup publishers
        self.driver = DaguWheelsDriver()
        self.msg_wheels_cmd = WheelsCmdStamped()

        # Publisher for wheels command wih execution time
        self.pub_wheels_cmd = self.publisher("~wheels_cmd_executed", WheelsCmdStamped, queue_size=1)

        # Subscribers
        self.sub_topic = self.subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        self.sub_e_stop = self.subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)

        self.log("Initialized.")

    def cbWheelsCmd(self, msg):
        """Callback that sets wheels' speeds.

            Creates the wheels' speed message and publishes it. If the
            emergency stop flag is activated, publishes zero command.

            Args:
                msg (WheelsCmdStamped): velocity command
        """

        if self.estop:
            self.driver.setWheelsSpeed(left=0.0, right=0.0)
            return

        self.driver.setWheelsSpeed(left=msg.vel_left, right=msg.vel_right)
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()
        self.msg_wheels_cmd.vel_left = msg.vel_left
        self.msg_wheels_cmd.vel_right = msg.vel_right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def cbEStop(self, msg):
        """Callback that enables/disables emergency stop

            Args:
                msg (BoolStamped): emergency_stop flag
        """

        self.estop = msg.data
        if self.estop:
            rospy.log("Emergency Stop Activated")
        else:
            rospy.log("Emergency Stop Released")

    def on_shutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""

        self.driver.setWheelsSpeed(left=0.0, right=0.0)

        super(WheelsDriverNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node with rospy
    node = WheelsDriverNode(node_name='wheels_driver_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
