#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8, String
from rgb_led import RGB_LED
from duckietown_msgs.msg import BoolStamped, LEDPattern
from led_emitter.srv import SetCustomLED, ChangePattern
from led_emitter.srv import SetCustomLEDResponse, ChangePatternResponse


class LEDEmitterNode(object):
    """Class for LEDs managment.

            Calls the low-level functions of class RGB_LED that create the PWM
            signal used to steer the LEDs. According to the requested pattern
            (pattern = color + frequency), changes the frequency of calling the
            update and the color.

            Subscribers:
                ~change_color_pattern (String): A string that...

                ~switch (BoolStamped): A BoolStamped that...

                ~custom_pattern (LEDPattern): An LEDPattern that...

            Publishers:
                ~current_led_state (String): A string that...

            Services:
                ~change_led (SetCustomLED): Description of the service

                ~set_pattern (ChangePattern): Description of the service

            """

    def __init__(self):

        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing..." % (self.node_name))

        self.led = RGB_LED()

        self.active = True

        # Import protocol
        self.protocol = rospy.get_param("~LED_protocol")

        # Import parameters
        self.scale = rospy.get_param("~LED_scale")

        # Initialize LEDs to be off
        self.pattern = [[0, 0, 0]]*5
        self.current_pattern_name = 'OFF'
        self.changePattern_(self.current_pattern_name)

        # Initialize the timer
        self.frequency = 1.0/self.protocol['signals']['CAR_SIGNAL_A']['frequency']
        self.is_on = False
        self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(self.frequency/(2.0)),
                                       self.cycleTimer_)

        # Publishers
        self.pub_state = rospy.Publisher("~current_led_state",
                                         String,
                                         queue_size=1)
        # Subscribers
        self.sub_pattern = rospy.Subscriber("~change_color_pattern",
                                            String,
                                            self.changePattern)
        self.sub_switch = rospy.Subscriber("~switch",
                                           BoolStamped,
                                           self.cbSwitch)
        self.sub_custom_pattern = rospy.Subscriber("~custom_pattern",
                                                   LEDPattern,
                                                   self.cbCustomPattern)
        # Services
        self.srv_set_LED_ = rospy.Service("~change_led",
                                          SetCustomLED,
                                          self.srvSetCustomLED)
        self.srv_set_pattern_ = rospy.Service("~set_pattern",
                                              ChangePattern,
                                              self.srvSetPattern)

        # Scale intensity of the LEDs
        for _, c in self.protocol['colors'].items():
            for i in range(3):
                c[i] = c[i] * self.scale

        # Turn on the LEDs
        self.changePattern_('WHITE')

        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def srvSetCustomLED(self, LED_pattern):
        """Service to set a custom pattern.

            Sets the LEDs to a custom pattern (colors+frequency)

            Args:
                LED_pattern (LEDPattern): requested pattern

            Example:

                To do bla::

                .......([....],12)
        """
        rospy.loginfo("Changing LEDs to custom pattern")
        self.current_pattern_name = 'custom_pattern'
        self.pattern = LED_pattern.pattern

        if self.frequency == 0:
            self.updateLEDs()
        else:
            self.changeFrequency()
        return SetCustomLEDResponse()

    def cbCustomPattern(self, msg_pattern):
        """Changes LEDs to received custom pattern.

            Callback that updates the current object according to the message
            received. After that calls the uptadeLEDs function, that sends
            the signal to the LEDs.

            Args:
                msg_pattern (LEDPattern): requested pattern
        """
        self.color_list = msg_pattern.color_list
        self.color_mask = msg_pattern.color_mask
        self.frequency = msg_pattern.frequency
        self.frequency_mask = msg_pattern.frequency_mask
        self.updateLEDs()

    def cbSwitch(self, switch_msg):
        """Callback that turns on/off the node.

            Reads the switch from the Finite State Machine and sets
            self.active accordingly.

            Args:
                switch_msg (BoolStamped): Switch for the node.
        """
        self.active = switch_msg.data

    def cycleTimer_(self, event):
        """Timer.

            Calls updateLEDs according to the frequency of the current pattern.

            Args:
                event (TimerEvent): event generated by the timer.
        """
        self.updateLEDs()

    def updateLEDs(self):
        """Switches the LEDs to the requested signal.

            If the pattern is static, changes the color of LEDs according to
            the color specified in self.color_list on the LEDs specified
            on self.color_mask. If a nonzero frequency is set, toggles on/off
            the LEDs specified on self.frequency_mask.
        """

        # Do nothing if inactive
        if not self.active:
            return

        elif not self.frequency:
            # No oscillation
            for i in range(5):
                if self.color_mask[i] == True:
                    colors = self.pattern[i]
                    self.led.setRGB(i, colors)
        else:
            # Oscillate
            if self.is_on:
                for i in range(5):
                    if self.frequency_mask[i] == True:
                        self.led.setRGB(i, [0, 0, 0])
                self.is_on = False

            else:
                for i in range(5):
                    colors = self.pattern[i]
                    self.led.setRGB(i, colors)
                self.is_on = True

    def srvSetPattern(self, msg):
        """Changes the current pattern according to msg.

            Args:
                msg (String): requested pattern name
        """
        self.changePattern_(msg.data)
        return ChangePatternResponse()

    def changePattern(self, msg):
        """Calls the private function with data from message"""
        self.changePattern_(msg.data)

    def changePattern_(self, pattern_name):
        """Change the current LED pattern.

            Checks if the requested pattern is different from the current one,
            if so changes colors and frequency of LEDs accordingly and
            publishes the new current pattern.

            Args:
                pattern_name (string): Name of the wanted pattern

        """
        if pattern_name:
            # No need to change if we already have the right pattern
            if self.current_pattern_name == pattern_name:
                return
            else:
                self.current_pattern_name = pattern_name

            # Extract the color from the protocol config file
            color_mask = self.protocol['signals'][pattern_name]['color_mask']
            color_list = self.protocol['signals'][pattern_name]['color_list']

            # If color mask is not specified, we want to change all of the LEDs
            if len(color_mask) < 5:
                self.color_mask = [1]*5
            else:
                self.color_mask = color_mask

            # Extract the frequency from the protocol
            self.frequency_mask = self.protocol['signals'][pattern_name]['frequency_mask']
            self.frequency = self.protocol['signals'][pattern_name]['frequency']

            if type(color_list) is str:
                self.pattern = [self.protocol['colors'][color_list]]*5
            else:
                assert (len(color_list) == 5), "color_list needs 5 entries"
                self.pattern = [[0, 0, 0]]*5
                for i in range(len(color_list)):
                    self.pattern[i] = self.protocol['colors'][color_list[i]]

            # If static behavior, updated LEDs
            if self.frequency == 0:
                self.updateLEDs()

            # Anyway modify the frequency (to stop timer if static)
            self.changeFrequency()

            # Loginfo
            rospy.loginfo('[%s] Pattern changed to (%r), cycle: %s '
                          % (self.node_name, pattern_name, self.frequency))

            # Publish current pattern
            self.pub_state.publish(self.current_pattern_name)

    def changeFrequency(self):
        """Changes current frequency of LEDs

            Stops the current cycle_timer, and starts a new one with the
            frequency specified in self.frequency. If the frequency is zero,
            stops the callback timer.
        """
        if self.frequency == 0:
            self.cycle_timer.shutdown()

        else:
            try:
                self.cycle_timer.shutdown()
                # below, convert to hz
                d = 1.0/(2.0*self.frequency)
                self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(d), self.cycleTimer_)

            except ValueError as e:
                self.frequency = None
                self.current_pattern_name = None

    def onShutdown(self):
        """Shutdown procedure.

        At shutdown, changes the LED pattern to `light_off`."""

        # Turn off the lights when the node dies
        self.changePattern_('light_off')

        rospy.loginfo("[%s] Shutting down." % (rospy.get_name()))


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('led_emitter', anonymous=False)
    # Create the LEDEmitterNode object
    led_emitter_node = LEDEmitterNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(led_emitter_node.onShutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
