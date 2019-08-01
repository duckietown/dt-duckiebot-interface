#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8
from duckietown_msgs.msg import LEDPattern


class TrafficLightNode(object):
    """Handles the LED patterns for the traffic lights.

    The class creates an object which handles the timers for managing
    a traffic light at an intersection. By default a 4-way intersection
    is assumed, this can be modified by the `self.number_leds` parameter.
    The LED protocol used is the same as LED emitter node, for coherence.
    An additional protocol defines the lenght of the periods of green light
    and the order of the directions. The changes are made by publishing
    to a special topic of LedEmitterNode.

    Publishers:
        ~custom_pattern (LEDPattern): Description
    """

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing..." % (self.node_name))

        # Import protocols
        self.LED_protocol = rospy.get_param("~LED_protocol")
        self.TL_protocol = rospy.get_param("~TL_protocol")
        self.scale = rospy.get_param("~LED_scale")

        # Setup parameters
        self.green_frequency = self.LED_protocol['signals']['TL_GO']['frequency']
        self.green_light_duration = self.TL_protocol['timings']['green_time']
        self.all_red_duration = self.TL_protocol['timings']['all_red_time']
        self.number_leds = self.TL_protocol['leds']['number_leds']
        self.activation_order = self.TL_protocol['leds']['activation_order']

        self.color_mask = [0]*5
        self.color_mask[0:self.number_leds-1] = [1]*self.number_leds
        self.green_idx = 0

        cycle_duration = self.green_light_duration + self.all_red_duration

        # Publishers
        self.pub_pattern = rospy.Publisher("~custom_pattern",
                                           LEDPattern,
                                           queue_size=1)
        # Initialize timer for direction change
        self.traffic_cycle = rospy.Timer(rospy.Duration(cycle_duration),
                                         self.change_direction)
        # Scale intensity of the LEDs & convert to right order
        for _, c in self.LED_protocol['colors'].items():
            # RGB to BGR (TL currently has BGR leds)
            c[1], c[3] = c[3], c[1]
            for i in range(3):
                c[i] = c[i] * self.scale

        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def change_direction(self, event):
        """Callback changing direction of green light.

            The callback iterates through the LEDs of a traffic light and
            generates an LEDPattern message accordingly. The green light blinks
            at the defined frequency, while the red light stays static. After
            the green light blinks according to protocol, there is a dead zone
            in which all directions are on red. The message is published as a
            special pattern to the led_emitter_node.
        """
        # Move to next light in list
        self.green_idx = (self.green_idx + 1) % (self.number_leds)
        self.green_LED = self.activation_order[self.green_idx]

        # Only blink the green LED
        self.frequency_mask = [0]*5
        self.frequency_mask[self.green_LED] = 1

        # Create Protocol (we fake 5 LEDs, but the last will not be updated)
        self.color_list = [self.LED_protocol['colors']['red']] * 5
        self.color_list[self.green_LED] = self.LED_protocol['colors']['green']

        # Build message
        pattern_msg = LEDPattern()
        pattern_msg.color_list = self.color_list
        pattern_msg.color_mask = self.color_mask
        pattern_msg.frequency = self.green_frequency
        pattern_msg.frequency_mask = self.frequency_mask
        self.pub_pattern.publish(pattern_msg)

        # Keep the green light on
        rospy.sleep(self.green_light_duration)

        # Turn all on red for safety
        pattern_msg.color_list = [self.LED_protocol['colors']['red']] * 5
        pattern_msg.frequency = 0
        self.pub_pattern.publish(pattern_msg)

    def onShutdown(self):
        """Shutdown procedure."""
        rospy.loginfo("[%s] Shutting down." % (rospy.get_name()))


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('traffic_light', anonymous=False)
    # Create the TrafficLightNode object
    traffic_light_node = TrafficLightNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(traffic_light_node.onShutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
