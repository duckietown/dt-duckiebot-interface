#!/usr/bin/env python
import rospy
import math

from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy

from __builtin__ import True
from easy_node.easy_node import EasyNode

class JoyMapperNode2(EasyNode):
    def __init__(self):
        EasyNode.__init__(self, 'joy_mapper', 'joy_mapper_node2')
        self.info("Initializing")
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()
        
    def on_parameters_changed(self, first_time, _updated):
        if first_time:
                
            self.has_complained = False
    
            self.state_parallel_autonomy = False
            self.state_verbose = False            
        
            pub_msg = BoolStamped()
            pub_msg.data = self.state_parallel_autonomy
            pub_msg.header.stamp = self.last_pub_time
            self.publishers.parallel_autonomy.publish(pub_msg)
        else:
            pass 

    def on_received_joy(self, _context, joy_msg):
        self.publish_control(joy_msg)
        self.process_buttons(joy_msg)

    def publish_control(self, joy_msg):
        # Configuration parameters
        v_gain = self.config.v_gain
        omega_gain = self.config.omega_gain
        steer_angle_gain = self.config.steer_angle_gain
        simulated_vehicle_length = self.config.simulated_vehicle_length
        
        # Input data
        # Left stick V-axis. Up is positive
        cmd_vel = joy_msg.axes[1]
        cmd_steer = joy_msg.axes[3]
        
        # Do the computation
        v = cmd_vel * v_gain 
        
        if self.config.bicycle_kinematics:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            # see: https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = cmd_steer * steer_angle_gain
            omega = v / simulated_vehicle_length * math.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            omega = cmd_steer * omega_gain
            
        # Create the message and publish
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = joy_msg.header.stamp
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega 
        self.publishers.car_cmd.publish(car_cmd_msg)
    
    def process_buttons(self, joy_msg):
        BT_A = 0  # @UnusedVariable
        BT_B = 1  # @UnusedVariable
        BT_X = 2  # @UnusedVariable
        BT_Y = 3
        BT_LB = 4
        BT_RB = 5
        BT_BACK = 6
        BT_START = 7
        BT_LOGITEK = 8 # power button (middle)
        BT_LEFT_JOY = 9
        BT_RIGHT_JOY = 10  # @UnusedVariable
        
        if (joy_msg.buttons[BT_BACK] == 1): 
            self.info('override_msg = True')
            self._send_bool_stamped(self.publishers.joy_override, True, joy_msg)

        elif (joy_msg.buttons[BT_START] == 1):
            self.info('override_msg = False')
            self._send_bool_stamped(self.publishers.joy_override, False, joy_msg)

        elif (joy_msg.buttons[BT_RB] == 1): 
            self.state_verbose ^= True
            self.info('state_verbose = %s' % self.state_verbose)
            # bad - should be published for all to hear - not set a specific param
            rospy.set_param('line_detector_node/verbose', self.state_verbose) 

        elif (joy_msg.buttons[BT_LB] == 1):
            self.state_parallel_autonomy ^= True
            self.info('state_parallel_autonomy = %s' % self.state_parallel_autonomy)
            self._send_bool_stamped(self.publishers.parallel_autonomy, self.state_parallel_autonomy, joy_msg)

        elif (joy_msg.buttons[BT_Y] == 1):
            self.info('anti_instagram message')
            self._send_bool_stamped(self.publishers.anti_instagram, True, joy_msg)

        elif (joy_msg.buttons[BT_LOGITEK] == 1): 
            self.info('E-stop message')
            data = True # note that this is toggle (actual value doesn't matter)
            self._send_bool_stamped(self.publishers.e_stop, data, joy_msg)

        elif (joy_msg.buttons[BT_LEFT_JOY] == 1):
            self.info('start lane following with avoidance mode')
            self._send_bool_stamped(self.publishers.avoidance, True, joy_msg)

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                self.info('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))

    def _send_bool_stamped(self, publisher, value, reference_msg):
        m = BoolStamped()
        m.header.stamp = reference_msg.header.stamp
        m.data = value
        publisher.publish(m)


if __name__ == "__main__":
    JoyMapperNode2().spin()
