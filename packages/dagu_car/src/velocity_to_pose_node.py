#!/usr/bin/env python

import numpy as np
import rospy

from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped

class VelocityToPoseNode(DTROS):
    """
    VelocityToPoseNode integrates the velocity of the Duckiebot in order to continuously obtain a pose
    relative to the pose at which the node was started.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Subscriber:
        ~velocity (:obj:`Twist2DStamped`): The robot velocity, typically obtained from forward kinematics

    Publisher:
        ~pose (:obj:`Pose2DStamped`): The integrated pose relative to the pose of the robot at node initialization

    """
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(VelocityToPoseNode, self).__init__(node_name=node_name)

        # Get the vehicle name
        self.veh_name = rospy.get_namespace().strip("/")

        # Keep track of the last known pose
        self.last_pose = Pose2DStamped()
        self.last_theta_dot = 0
        self.last_v = 0
        
        # Setup the publisher and subscriber
        self.sub_velocity = self.subscriber("~velocity", Twist2DStamped, self.velocity_callback, queue_size=1)
        self.pub_pose = self.publisher("~pose", Pose2DStamped, queue_size=1)
        self.log("Initialized.")

    def velocity_callback(self, msg_velocity):
        """
        Performs the calclulation from velocity to pose and publishes a messsage with the result.

        Args:
            msg_velocity (:obj:`Twist2DStamped`): the current velocity message

        """
        if self.last_pose.header.stamp.to_sec() > 0:  # skip first frame

            dt = (msg_velocity.header.stamp - self.last_pose.header.stamp).to_sec()

            # Integrate the relative movement between the last pose and the current
            theta_delta = self.last_theta_dot * dt
            # to ensure no division by zero for radius calculation:
            if np.abs(self.last_theta_dot) < 0.000001:
                # straight line
                x_delta = self.last_v * dt
                y_delta = 0
            else:
                # arc of circle
                radius = self.last_v / self.last_theta_dot
                x_delta = radius * np.sin(theta_delta)
                y_delta = radius * (1.0 - np.cos(theta_delta))

            # Add to the previous to get absolute pose relative to the starting position
            theta_res = self.last_pose.theta + theta_delta
            x_res = self.last_pose.x + x_delta * np.cos(self.last_pose.theta) - y_delta * np.sin(self.last_pose.theta)
            y_res = self.last_pose.y + y_delta * np.cos(self.last_pose.theta) + x_delta * np.sin(self.last_pose.theta)

            # Update the stored last pose
            self.last_pose.theta = theta_res
            self.last_pose.x = x_res
            self.last_pose.y = y_res

            # Stuff the new pose into a message and publish
            msg_pose = Pose2DStamped()
            msg_pose.header = msg_velocity.header
            msg_pose.header.frame_id = self.veh_name
            msg_pose.theta = theta_res
            msg_pose.x = x_res
            msg_pose.y = y_res
            self.pub_pose.publish(msg_pose)

        self.last_pose.header.stamp = msg_velocity.header.stamp
        self.last_theta_dot = msg_velocity.omega
        self.last_v = msg_velocity.v


if __name__ == '__main__':
    # Initialize the node
    velocity_to_pose_node = VelocityToPoseNode(node_name='velocity_to_pose_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
