#!/usr/bin/env python
import rospy

from duckietown import DTROS

from rgb_led import RGB_LED
from std_msgs.msg import Float32, Int8, String
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
from duckietown_msgs.srv import SetCustomLEDPatternResponse, ChangePatternResponse


class LEDEmitterNode(DTROS):
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

            rosservice call /HOSTNAME/led_emitter_node/set_pattern "pattern_name: {data: RED}"

        Other pre-defined patterns you can use are: `WHITE`, `GREEN`, `BLUE`, `LIGHT_OFF`, `CAR_SIGNAL_PRIORITY`,
        `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`, `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`, `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`,
        `CAR_DRIVING`.

        To add a custom pattern and switch to it use a variant of the following::

            rosservice call /HOSTNAME/led_emitter_node/set_custom_pattern "pattern: {color_list: ['green','yellow','pink','orange','blue'], color_mask: [1,1,1,1,1], frequency: 1.0, frequency_mask: [1,0,1,0,1]}"


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
        super(LEDEmitterNode, self).__init__(node_name=node_name)

        self.led = RGB_LED()

        self.robot_type = rospy.get_param("~robot_type")

        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~LED_protocol'] = None
        self.parameters['~LED_scale'] = None
        self.parameters['~channel_order'] = None
        self.updateParameters()

        # Import protocol
        self.protocol = rospy.get_param("~LED_protocol")

        # Import parameters
        self.scale = rospy.get_param("~LED_scale")

        # Initialize LEDs to be off
        self.pattern = [[0, 0, 0]]*5
        self.frequency_mask = [0]*5
        self.current_pattern_name = 'LIGHT_OFF'
        self.changePattern(self.current_pattern_name)

        # Initialize the timer
        self.frequency = 1.0/self.parameters['~LED_protocol']['signals']['CAR_SIGNAL_A']['frequency']
        self.is_on = False
        self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(self.frequency/2.0),
                                       self.cycleTimer_)

        # Publishers
        self.pub_state = self.publisher("~current_led_state",
                                        String,
                                        queue_size=1)

        # Services
        self.srv_set_LED_ = rospy.Service("~set_custom_pattern",
                                          SetCustomLEDPattern,
                                          self.srvSetCustomLEDPattern)
        self.srv_set_pattern_ = rospy.Service("~set_pattern",
                                              ChangePattern,
                                              self.srvSetPattern)

        # Scale intensity of the LEDs
        for name, c in self.parameters['~LED_protocol']['colors'].items():
            for i in range(3):
                c[i] = c[i] * self.parameters['~LED_scale']

        # Remap colors if robot does not have an RGB ordering
        if self.parameters['~channel_order'][self.robot_type] is not "RGB":
            protocol = self.parameters['~LED_protocol']
            for name, col in self.parameters['~LED_protocol']['colors'].items():
                protocol['colors'][name] = self.remapColors(col)

            rospy.set_param("~LED_protocol", protocol)
            self.updateParameters()
            self.log("Colors remapped to " +
                     str(self.parameters['~channel_order'][self.robot_type]))

        # Turn on the LEDs
        self.changePattern('WHITE')

        self.log("Initialized.")

    def srvSetCustomLEDPattern(self, req):
        """Service to set a custom pattern.

            Sets the LEDs to a custom pattern. The :obj:`LEDPattern` message from
            :obj:`duckietown_msgs` is used for that.

            Args:
                LED_pattern (LEDPattern): the requested pattern

        """

        # Update the protocol
        protocol = self.parameters['~LED_protocol']
        protocol['signals']['custom'] = {'color_mask': req.pattern.color_mask,
                                         'color_list': req.pattern.color_list,
                                         'frequency_mask': req.pattern.frequency_mask,
                                         'frequency': req.pattern.frequency}
        # Set it through rosparam because otherwise the background parameter checker will overwrite it
        rospy.set_param("~LED_protocol", protocol)
        self.updateParameters(verbose=False)

        self.log("Custom pattern updated: color_mask: %s, color_list: %s, frequency_mask: %s, frequency: %s" %
                 (str(self.parameters['~LED_protocol']['signals']['custom']['color_mask']),
                  str(self.parameters['~LED_protocol']['signals']['custom']['color_list']),
                  str(self.parameters['~LED_protocol']['signals']['custom']['frequency_mask']),
                  str(self.parameters['~LED_protocol']['signals']['custom']['frequency'])))

        # Perform the actual change
        self.changePattern('custom')

        return SetCustomLEDPatternResponse()


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
            the color specified in self.color_list. If a nonzero frequency is set,
            toggles on/off the LEDs specified on self.frequency_mask.
        """

        # Do nothing if inactive
        if not self.switch:
            return

        elif not self.frequency:
            # No oscillation
            for i in range(5):
                colors = self.pattern[i]
                self.led.setRGB(i, colors)
        else:
            # Oscillate
            if self.is_on:
                for i in range(5):
                    if self.frequency_mask[i]:
                        self.led.setRGB(i, [0, 0, 0])
                self.is_on = False

            else:
                for i in range(5):
                    colors = self.pattern[i]
                    self.led.setRGB(i, colors)
                self.is_on = True

    def srvSetPattern(self, msg):
        """Changes the current pattern according to the pattern name sent in the message.


            Args:
                msg (String): requested pattern name
        """
        self.changePattern(str(msg.pattern_name.data))
        return ChangePatternResponse()

    def changePattern(self, pattern_name):
        """Change the current LED pattern.

            Checks if the requested pattern is different from the current one,
            if so changes colors and frequency of LEDs accordingly and
            publishes the new current pattern. If the requested pattern name
            is not found, it will not change the pattern and will publish ROS
            Error log message.

            Args:
                pattern_name (string): Name of the wanted pattern

        """
        if pattern_name:
            # No need to change if we already have the right pattern, unless it is other, because
            # we might have updated its definition
            if self.current_pattern_name == pattern_name and pattern_name != 'custom':
                return
            elif pattern_name.strip("'").strip('"') in self.parameters['~LED_protocol']['signals']:
                self.current_pattern_name = pattern_name
            else:
                self.log("Pattern name %s not found in the list of patterns. Change of "
                         "pattern not executed." % pattern_name, type='err')
                self.log(self.parameters['~LED_protocol']['signals'], type='err')
                return

            # Extract the color from the protocol config file
            color_list = self.parameters['~LED_protocol']['signals'][pattern_name]['color_list']

            if type(color_list) is str:
                self.pattern = [self.parameters['~LED_protocol']['colors'][color_list]]*5
            else:
                if len(color_list) != 5:
                    self.log("The color list should be a string or a list of length 5. Change of "
                             "pattern not executed.", type='err')
                    return

                self.pattern = [[0, 0, 0]]*5
                for i in range(len(color_list)):
                    self.pattern[i] = self.parameters['~LED_protocol']['colors'][color_list[i]]

            # Extract the frequency from the protocol
            self.frequency_mask = self.parameters['~LED_protocol']['signals'][pattern_name]['frequency_mask']
            self.frequency = self.parameters['~LED_protocol']['signals'][pattern_name]['frequency']

            # If static behavior, updated LEDs
            if self.frequency == 0:
                self.updateLEDs()

            # Anyway modify the frequency (to stop timer if static)
            self.changeFrequency()

            # Loginfo
            self.log('Pattern changed to (%r), cycle: %s ' % (pattern_name, self.frequency))

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

    def remapColors(self, color):
        """
        Remaps a color from RGB to the channel ordering currently set in the
        `channel_order` configuration parameter.

        Args:
            color (:obj:`list` of :obj:`float`): A color triplet

        Returns:
            :obj:`list` of :obj:`float`: The triplet with reordered channels

        """

        # Verify that the requested reordering is valid
        allowed_orderings = ['RGB', 'RBG', 'GBR', 'GRB', 'BGR', 'BRG']
        requested_ordering = self.parameters['~channel_order'][self.robot_type]
        if requested_ordering not in allowed_orderings:
            self.log("The current channel order %s is not supported, use one of %s. "
                     "The remapping was not performed." % \
                     (requested_ordering, str(allowed_orderings)), type='warn')
            return color

        reordered_triplet = list()
        rgb_map = {'R':0, 'G':1, 'B':2}
        for channel_color in requested_ordering:
            reordered_triplet.append(color[rgb_map[channel_color]])

        return reordered_triplet


    def onShutdown(self):
        """Shutdown procedure.

        At shutdown, changes the LED pattern to `LIGHT_OFF`."""

        # Turn off the lights when the node dies
        self.changePattern('LIGHT_OFF')

        super(LEDEmitterNode, self).onShutdown()


if __name__ == '__main__':
    # Create the LEDEmitterNode object
    led_emitter_node = LEDEmitterNode(node_name='led_emitter')
    # Keep it spinning to keep the node alive
    rospy.spin()
