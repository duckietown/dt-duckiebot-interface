#!/usr/bin/env python3
import time

import rospy

from rgb_led import RGB_LED
from std_msgs.msg import String, ColorRGBA
from duckietown_msgs.msg import LEDPattern
from duckietown.dtros import DTROS, TopicType, NodeType
from hardware_test_led import HWTestLED


class LEDDriverNode(DTROS):
    """Node for controlling LEDs.

    Calls the low-level functions of class :obj:`RGB_LED` that creates the PWM
    signal used to change the color of the LEDs. The desired behavior is specified by
    the LED index (Duckiebots and watchtowers have multiple of these) and a pattern.
    A pattern is a combination of colors and blinking frequency.

    Duckiebots have 5 LEDs that are indexed and positioned as following:

    +------------------+------------------------------------------+
    | Index            | Position (rel. to direction of movement) |
    +==================+==========================================+
    | 0                | Front left                               |
    +------------------+------------------------------------------+
    | 1                | Rear left                                |
    +------------------+------------------------------------------+
    | 2                | Top / Front middle                       |
    +------------------+------------------------------------------+
    | 3                | Rear right                               |
    +------------------+------------------------------------------+
    | 4                | Front right                              |
    +------------------+------------------------------------------+

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LEDDriverNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)

        self.led = RGB_LED()

        start_color = [0, 0, 0]
        for i in range(5):
            self.led.setRGB(i, start_color)

        self.sub_topic = rospy.Subscriber("~led_pattern", LEDPattern, self.led_cb, queue_size=1)

        self.hw_test_front = HWTestLED(self.led, front_leds=True)
        self.hw_test_back = HWTestLED(self.led, front_leds=False)
        self.log("Initialized.")

    def led_cb(self, msg):
        """Switches the LEDs to the requested signal."""

        for i in range(5):
            colors = [msg.rgb_vals[i].r, msg.rgb_vals[i].g, msg.rgb_vals[i].b]
            self.led.setRGB(i, colors)

    def on_shutdown(self):
        """Shutdown procedure.

        At shutdown, changes the LED pattern to `LIGHT_OFF`.
        """
        # Turn off the lights when the node dies
        self.loginfo("Shutting down. Turning LEDs off.")
        time.sleep(1)


if __name__ == "__main__":
    # Create the LEDdriverNode object
    led_driver_node = LEDDriverNode(node_name="led_driver_node")
    # Keep it spinning to keep the node alive
    rospy.spin()
