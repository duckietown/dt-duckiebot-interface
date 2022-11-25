#!/usr/bin/env python3
import time

import rospy

from rgb_led import RGB_LED
from std_msgs.msg import String, ColorRGBA
from duckietown_msgs.msg import LEDPattern
from duckietown.dtros import DTROS, TopicType, NodeType


class LEDDrivererNode(DTROS):
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

    A pattern is specified via 5 parameters:

    - its name

    - frequency: blinking frequency in Hz, should be set to 0 for a solid (non-blinking) behavior

    - color_list: a list of 5 colour names (see below), one for each LED ordered as above, or a single string with
      a single color name that would be applied to all LEDs

    - frequency_mask: a list of 5 binary flags (0 or 1) that specify which of the LEDs should be blinking,
      used only if the frequency is not 0. The LEDs with the flag set to 0, will maintain their solid color.

    The defaut patterns are defined in the `LED_protocol.yaml` configuration file for the node.

    Currently supported colors are: `green`, `red`, `blue`, `white`, `yellow`, `purple`, `cyan`,
    `pink`, `switchedoff`. More colors can be defined in the node's configuration file.

    Examples:

        To change the pattern to one of the predefined patterns (you can see them using `rosparam list`)
        use a variant of the following::

            rosservice call /HOSTNAME/led_driver_node/set_pattern "pattern_name: {data: RED}"

        Other pre-defined patterns you can use are: `WHITE`, `GREEN`, `BLUE`, `LIGHT_OFF`, `CAR_SIGNAL_PRIORITY`,
        `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`, `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`, `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`,
        `CAR_DRIVING`.

        To add a custom pattern and switch to it use a variant of the following::

            rosservice call /HOSTNAME/led_driver_node/set_custom_pattern "pattern: {color_list: ['green','yellow','pink','orange','blue'], color_mask: [1,1,1,1,1], frequency: 1.0, frequency_mask: [1,0,1,0,1]}"


    Configuration:
        ~LED_protocol (nested dictionary): Nested dictionary that describes the LED protocols (patterns). The
            default can be seen in the `LED_protocol.yaml` configuration file for the node.
        ~LED_scale (:obj:`float`): A scaling factor (between 0 and 1) that is applied to the colors in order
            to reduce the overall LED brightness, default is 0.8.
        ~channel_order (:obj:`str`): A string that controls the order in which the 3 color channels should be
            communicated to the LEDs. Should be one of 'RGB', `RBG`, `GBR`, `GRB`, `BGR`, `BRG`. Typically
            for a duckiebot this should be the default `RGB` and for traffic lights should be `GRB`, default is `RGB`.

    Publishers:
        ~current_led_state (:obj:`String` message): Publishes the name of the current pattern used. Published
            only when the selected pattern changes.

    Services:
        ~set_custom_pattern: Allows setting a custom protocol. Will be named `custom`. See an example of a call
            in :obj:`srvSetCustomLEDPattern`.

            input:

                pattern (:obj:`LEDPattern` message): The desired new LEDPattern

        ~set_pattern: Switch to a different pattern protocol.

            input:

                pattern_name (:obj:`String` message): The new pattern name, should match one of the patterns in
                   the `LED_protocol` parameter (or be `custom` if a custom pattern has been defined via a call to
                   the `~change_led` service.

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LEDdriverNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)

        self.led = RGB_LED()

        self.sub_topic = rospy.Subsriber("~led_pattern", LEDPattern, self.led_cb, queue_size=1)

        self.log("Initialized.")

    def led_cb(self, msg):
        """Switches the LEDs to the requested signal."""

        for i in range(5):
            colors = [msg.rgb_vals[i].r, msg.rgb_vals[i].g, msg.rg_vals[i].b]
            self.led.setRGB(i, colors)

    def on_shutdown(self):
        """Shutdown procedure.

        At shutdown, changes the LED pattern to `LIGHT_OFF`.
        """
        # Turn off the lights when the node dies
        self.loginfo("Shutting down. Turning LEDs off.")
        self.changePattern("LIGHT_OFF")
        time.sleep(1)


if __name__ == "__main__":
    # Create the LEDdriverNode object
    led_driver_node = LEDdriverNode(node_name="led_driver_node")
    # Keep it spinning to keep the node alive
    rospy.spin()
