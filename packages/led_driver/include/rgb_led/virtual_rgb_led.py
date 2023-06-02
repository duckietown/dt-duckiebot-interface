#!/usr/bin/env python3

from threading import Condition
from typing import List, Optional

from dt_duckiematrix_protocols.robot.features.lights import Lights
from dt_duckiematrix_utils.ros import \
    on_duckiematrix_connection_request, \
    DuckiematrixLinkDescription
from dt_robot_utils import get_robot_configuration

from dt_duckiematrix_protocols import Matrix

class VirtualRGBLED(object):
    """Object communicating to the LEDs.

    Low level class that creates the PWM messages that are sent to the
    microcontroller. It contains offset addresses relatives to the
    address of the various LEDs.

    Each LED on a Duckiebot or a watchtower is indexed by a number:

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

    Setting the color of a single LED is done by setting the brightness of the
    red, green, and blue channels to a value between 0 and 255. The communication
    with the hardware controller is abstracted through the :obj:`setRGB` method. By
    using it, you can set directly set the desired color to any LED.

    """

    def __init__(self, debug=False):
        
        self._matrix: Optional[Matrix] = None
        self._device: Optional[Lights] = None
        # register connection setup function
        self._connection_request: Optional[DuckiematrixLinkDescription] = None
        self._new_connection_request = Condition()
        on_duckiematrix_connection_request(self.on_connection_request)
    
    def on_connection_request(self, link: DuckiematrixLinkDescription):
        self.log(f"[VirtualLEDs]: Received request to connect to Duckiematrix '{link.matrix}'.")
        # store new connection request
        self._connection_request = link
        # notify node of the new connection
        with self._new_connection_request:
            self._new_connection_request.notify()

    def setup(self):
        if self._connection_request is None:
            print("Not connecting")
            return
        # ---
        link = self._connection_request
        configuration = get_robot_configuration()
        # create connection to the matrix engine
        self._matrix: Matrix = Matrix(link.uri)
        # create connection to the vehicle
        self.robot: Lights = self._matrix.robots.create(configuration.name, link.entity)

    def setLEDBrightness(self, led, offset, brightness):
        """Used in physical robot led driver

        Args:
            led (:obj:`int`): Index of specific LED (from the table above)
            offset (:obj:`int`): Offset for color
            brightness (:obj:`int8`): Intensity of brightness (between 0 and 255)

        """
        raise NotImplementedError
    
    def setRGB(self, led, color):
        """Sets value for brightness for all channels of one LED

        Converts the input color brightness from [0,1] to [0,255] for all
        channels, then sets the corresponding led channels.

        Args:
            led (:obj:`int`): Index of specific LED (from the table above)
            color (:obj:`list` of :obj:`float`): Brightness for the three RGB channels, in interval [0,1]
        """
        return
        # Setup the connection to the LEDs
        self.setup()
        # convert lights dictionary to an indexed list
        self.lights_list = self.get_lights_list()

        light_key = f"light{led}"

        with self.robot.lights.atomic():
            light = getattr(self.robot.lights,light_key)

            light.color.r = int(color[0] * 255)
            light.color.g = int(color[1] * 255)
            light.color.r = int(color[2] * 255)

    def __del__(self):
        """Destructor method.

        Turns off all the LEDs and deletes the PWM object.

        """
        with self.robot.lights.atomic():
            for light in self.lights_list:
                light.color.r = 0
                light.color.g = 0
                light.color.b = 0

        del self.lights_list
    
    def get_lights_list(self) -> List:
        lights_list = [
            self.robot.lights.light0,
            self.robot.lights.light1,
            self.robot.lights.light2,
            self.robot.lights.light3,
            self.robot.lights.light4,
        ]

        return lights_list
