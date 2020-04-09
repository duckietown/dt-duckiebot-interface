#!/usr/bin/env python
import rospy

from duckietown import DTROS

from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern


class TrafficLightNode(DTROS):
    """Handles the LED patterns for the traffic lights.

    The class creates an object which handles the timers for managing
    a traffic light at an intersection. By default a 4-way intersection
    is assumed, this can be modified by the `~number_leds` configuration parameter.
    The actual execution of the protocol, and the communication with the driver is done
    by :obj:`LEDEmitterNode`, so it must be running for the correct operation of this node.
    An additional protocol defines the length of the periods of green light
    and the order of the directions. All configuration parameters are dynamically configurable
    and can be updated via `rosparam set`.

    Configuration:
        ~number_leds (:obj:`int`): Number of LEDs, should be 3 or 4 depending on the
            intersection type, default is 4
        ~activation_order (:obj:`list` of :obj:`int`): The order of activation of the
            LEDs, default is [0,1,2,3]
        ~green_time (:obj:`float`): Duration of the green signal in seconds, default is 5
        ~all_red_time (:obj:`float`): Duration of a signal when all direction are red (waiting
            for the Duckiebots to clear the intersection) in seconds, default is 4
        ~frequency (:obj:`float`): The blinking frequency of the green signal, default is 7.8

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(TrafficLightNode, self).__init__(node_name=node_name)

        # Import protocols
        self.parameters["~number_leds"] = None
        self.parameters["~activation_order"] = None
        self.parameters["~green_time"] = None
        self.parameters["~all_red_time"] = None
        self.parameters["~frequency"] = None
        self.updateParameters()

        self.green_idx = 0

        cycle_duration = self.parameters["~green_time"] + self.parameters["~all_red_time"]

        # Create the color mask
        self.color_mask = [0]*5
        self.color_mask[0:self.parameters["~number_leds"]] = [1]*self.parameters["~number_leds"]

        # Function mapping to LEDEmitterNode's `set_custom_pattern` service
        self.changePattern = rospy.ServiceProxy(rospy.get_namespace()+'led_emitter_node/set_custom_pattern',
                                                SetCustomLEDPattern)

        # Start a timer that will regularly call a method that changes
        # the direction that get green light
        self.traffic_cycle = rospy.Timer(rospy.Duration(cycle_duration),
                                         self.change_direction)

        self.log("Initialized.")

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
        self.green_idx = (self.green_idx + 1) % (self.parameters["~number_leds"])
        green_LED = self.parameters["~activation_order"][self.green_idx]

        # Only blink the green LED
        frequency_mask = [0]*5
        frequency_mask[green_LED] = 1

        # Create Protocol (we fake 5 LEDs, but the last will not be updated)
        color_list = ['red'] * 5
        color_list[green_LED] = 'green'

        # Build message
        pattern_msg = LEDPattern()
        pattern_msg.color_list = self.to_led_order(color_list)
        pattern_msg.color_mask = self.color_mask
        pattern_msg.frequency = self.parameters["~frequency"]
        pattern_msg.frequency_mask = self.to_led_order(frequency_mask)

        self.changePattern(pattern_msg)

        # Keep the green light on
        rospy.sleep(self.parameters["~green_time"])

        # Turn all on red for safety
        pattern_msg.color_list = ['red'] * 5
        pattern_msg.frequency = 0
        self.changePattern(pattern_msg)

        # If the parameters have been changed, update them and reset the timer.
        if self.parametersChanged:

            # Change the flag so that we don't update them again
            self.parametersChanged = False

            # Update the color mask
            self.color_mask = [0] * 5
            self.color_mask[0:self.parameters["~number_leds"]] = [1] * self.parameters["~number_leds"]

            # Update the cycle duration and restart the timer
            cycle_duration = self.parameters["~green_time"] + self.parameters["~all_red_time"]
            self.traffic_cycle.shutdown()
            self.traffic_cycle = rospy.Timer(rospy.Duration(cycle_duration), self.change_direction)

    def to_led_order(self, unordered_list):
        """Change ordering from successive (0,1,2,3,4) to the one expected by the led emitter (0,4,1,3,2)

            Args:
                unordered_list (:obj:`list`): List to be ordered.
            Returns:
                :obj: `list`: Permutated list of length ~number_leds.
        """
        ordering = [0, 4, 1, 3, 2]
        ordered_list = [unordered_list[i] for i in ordering]
        return ordered_list


if __name__ == '__main__':
    # Initialize the node
    traffic_light_node = TrafficLightNode(node_name='traffic_light')
    # Keep it spinning to keep the node alive
    rospy.spin()
