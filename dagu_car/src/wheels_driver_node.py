#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from dagu_car.dagu_wheels_driver import DaguWheelsDriver
import numpy as np

class WheelsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.estop=False

        # Parameters for maximal turning radius
        self.use_rad_lim        =   self.setupParam("~use_rad_lim", False)
        self.min_rad            =   self.setupParam("~min_rad", 0.08)
        self.wheel_distance     =   self.setupParam("~wheel_distance", 0.103)


        # Setup publishers
        self.driver = DaguWheelsDriver()
        #add publisher for wheels command wih execution time
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_executed",WheelsCmdStamped, queue_size=1)

        # Setup subscribers
        self.control_constant = 1.0
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)
        self.sub_rad_lim = rospy.Subscriber("~radius_limit", BoolStamped, self.cbRadLimit, queue_size=1)

        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def updateParams(self,event):
        self.use_rad_lim        =   rospy.get_param("~use_rad_lim")
        self.min_rad            =   rospy.get_param("~min_rad")
        self.wheel_distance     =   rospy.get_param("~wheel_distance")

    def cbWheelsCmd(self,msg):
        if self.estop:
            self.driver.setWheelsSpeed(left=0.0,right=0.0)
            return

        # Check if radius limitation is enabled
        if (self.use_rad_lim and (msg.vel_left != 0 or msg.vel_right != 0)):
            self.checkAndAdjustRadius(msg)


        self.driver.setWheelsSpeed(left=msg.vel_left,right=msg.vel_right)
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()
        self.msg_wheels_cmd.vel_left = msg.vel_left
        self.msg_wheels_cmd.vel_right = msg.vel_right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def cbRadLimit(self, msg):
        rospy.set_param("~use_rad_lim", msg.data)
        self.use_rad_lim = msg.data

    def checkAndAdjustRadius(self, msg):
        didAdjustment = False
        # if both motor cmds do not have the same sign, we're demanding for an on-point turn (not allowed)
        if (np.sign(msg.vel_left) != np.sign(msg.vel_right)):

            # Simply set the smaller velocity to zero
            if (abs(msg.vel_left) < abs(msg.vel_right)):
                msg.vel_left = 0.0
            else:
                msg.vel_right = 0.0

            didAdjustment = True

        # set v1, v2 from msg velocities such that v2 > v1
        if (abs(msg.vel_right) > abs(msg.vel_left)):
            v1 = msg.vel_left
            v2 = msg.vel_right
        else:
            v1 = msg.vel_right
            v2 = msg.vel_left

        # Check if a smaller radius than allowed is demanded
        if (v1 == 0 or abs(v2 / v1) > (self.min_rad + self.wheel_distance/2.0)/(self.min_rad - self.wheel_distance/2.0)):

            # adjust velocities evenly such that condition is fulfilled
            delta_v = (v2-v1)/2 - self.wheel_distance/(4*self.min_rad)*(v1+v2)
            v1 += delta_v
            v2 -= delta_v
            didAdjustment = True

        # set msg velocities from v1, v2 with the same mapping as when we set v1, v2
        if (abs(msg.vel_right) > abs(msg.vel_left)):
            msg.vel_left = v1
            msg.vel_right = v2
        else:
            msg.vel_left = v2
            msg.vel_right = v1

        return didAdjustment


    def cbEStop(self,msg):
        self.estop=not self.estop
        if self.estop:
            rospy.loginfo("[%s] Emergency Stop Activated")
        else:
            rospy.loginfo("[%s] Emergency Stop Released")

    def on_shutdown(self):
        self.driver.setWheelsSpeed(left=0.0,right=0.0)
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_driver_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsDriverNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
