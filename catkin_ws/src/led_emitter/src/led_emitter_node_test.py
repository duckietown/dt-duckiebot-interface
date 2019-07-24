#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8
import random


class LEDEmitterTest(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Subscribers
        self.sub_state = rospy.Subscriber("~current_led_state",
                                          Float32,
                                          self.changeState)
        # Publishers
        self.pub_state = rospy.Publisher("~change_light_frequency",
                                         Float32,
                                         queue_size=10)
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(5.0),
                                     self.cycleTimer)
        # Load LED protocol
        self.protocol = rospy.get_param("~LED_protocol")
        self.frequency_list = self.protocol[frequency]
        self.counter = 0

    def cycleTimer(self, event):
        self.pub_state.publish(self.frequency_list[self.counter])
        rospy.loginfo("Testing state " + str(self.state_list[self.counter]))
        self.counter = (self.counter+1) % len(self.state_list)

    def changeState(self, msg):
        rospy.loginfo("I see " + str(msg.data) + " as state")


if __name__ == '__main__':
    rospy.init_node('led_emitter', anonymous=False)
    node = LEDEmitterTest()
    rospy.spin()
