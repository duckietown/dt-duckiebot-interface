#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from wheels_driver.dagu_wheels_driver import DaguWheelsDriver


class WheelsDriverNode(object):
    """Sends commands to the motors."""

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing..." % (self.node_name))
        self.estop = False

        # Setup publishers
        self.driver = DaguWheelsDriver()
        self.msg_wheels_cmd = WheelsCmdStamped()

        # Publisher for wheels command wih execution time
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_executed", WheelsCmdStamped, queue_size=1)

        # Subscribers
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)

        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbWheelsCmd(self, msg):
        """Callback that sends motor command and publishes them with their
        timestamp
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
        """Callback that enables/disables emergency stop"""

        self.estop = msg.data
        if self.estop:
            rospy.loginfo("[%s] Emergency Stop Activated")
        else:
            rospy.loginfo("[%s] Emergency Stop Released")

    def on_shutdown(self):
        """Shutdown procedure"""

        self.driver.setWheelsSpeed(left=0.0, right=0.0)
        rospy.loginfo("[%s] Shutting down." % (rospy.get_name()))


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_driver_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsDriverNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
