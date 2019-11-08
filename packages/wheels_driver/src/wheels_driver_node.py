#!/usr/bin/env python
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from wheels_driver.dagu_wheels_driver import DaguWheelsDriver


class WheelsDriverNode(DTROS):
    """Node handling the motor velocities communication.

        Subscribes to the requested wheels commands (linear velocities, i.e. velocity for the left and
        the right wheels) and to an emergency stop flag. When the emergency flag `~emergency_stop` is set to
        `False` it actuates the wheel driver with the velocities received from `~wheels_cmd`. Publishes
        the execution of the commands to `~wheels_cmd_executed`.

        The emergency flag is `False` by default.

        Subscribers:
           ~wheels_cmd (:obj:`WheelsCmdStamped`): The requested wheel command
           ~emergency_stop (:obj:`BoolStamped`): Emergency stop. Can stop the actual execution of the
               wheel commands by the motors if set to `True`. Set to `False` for nominal operations.
        Publishers:
           ~wheels_cmd_executed (:obj:`WheelsCmdStamped`): Publishes the actual commands executed, i.e. when
               the emergency flag is `False` it publishes the requested command, and when it is `True`: zero
               values for both motors.

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(WheelsDriverNode, self).__init__(node_name=node_name)

        self.estop = False

        # Setup the driver
        self.driver = DaguWheelsDriver()

        # Initialize the executed commands message
        self.msg_wheels_cmd = WheelsCmdStamped()

        # Publisher for wheels command wih execution time
        self.pub_wheels_cmd = self.publisher(
            "~wheels_cmd_executed", WheelsCmdStamped, queue_size=1)

        # Subscribers
        self.sub_topic = self.subscriber(
            "~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        self.sub_e_stop = self.subscriber(
            "~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)

        self.log("Initialized.")

    def cbWheelsCmd(self, msg):
        """Callback that sets wheels' speeds.

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

        self.driver.setWheelsSpeed(left=vel_left, right=vel_right)
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()
        self.msg_wheels_cmd.vel_left = vel_left
        self.msg_wheels_cmd.vel_right = vel_right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def cbEStop(self, msg):
        """Callback that enables/disables emergency stop

            Args:
                msg (BoolStamped): emergency_stop flag
        """

        self.estop = msg.data
        if self.estop:
            self.log("Emergency Stop Activated")
        else:
            self.log("Emergency Stop Released")

    def onShutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""

        self.driver.setWheelsSpeed(left=0.0, right=0.0)

        super(WheelsDriverNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node with rospy
    node = WheelsDriverNode(node_name='wheels_driver_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
