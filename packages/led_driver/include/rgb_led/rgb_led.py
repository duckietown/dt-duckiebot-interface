#!/usr/bin/env python3
from typing import Tuple, List, Union

from Adafruit_PWM_Servo_Driver import PWM


class RGB_LED(object):
    """Object communicating to the LEDs.

    Low level class that creates the PWM messages that are sent to the
    microcontroller. It contains offset addresses relatives to the
    address of the various LEDs.

    Each LED on a Duckiebot or a watchtower is indexed by a number:

        +------------------+------------------+------------------------------------------+
        | Vehicle light    | Physical LED #   | Position on vehicle                      |
        +==================+==================+==========================================+
        | 0                | 0                | Front left                               |
        +------------------+------------------+------------------------------------------+
        | 1                | 2                | Rear left                                |
        +------------------+------------------+------------------------------------------+
        | 2                | 4                | Top / Front middle                       |
        +------------------+------------------+------------------------------------------+
        | 3                | 3                | Rear right                               |
        +------------------+------------------+------------------------------------------+
        | 4                | 1                | Front right                              |
        +------------------+------------------+------------------------------------------+

    Setting the color of a single LED is done by setting the brightness of the
    red, green, and blue channels to a value between 0 and 255. The communication
    with the hardware controller is abstracted through the :obj:`setRGB` method. By
    using it, you can set directly set the desired color to any LED.

    """

    # Class-specific constants
    OFFSET_RED = 0  #: Offset address for the red color
    OFFSET_GREEN = 1  #: Offset address for the green color
    OFFSET_BLUE = 2  #: Offset address for the blue color

    VEHICLE_TO_PHYSICAL_LED_MAPPING = {
        0: 0,
        1: 2,
        2: 4,
        3: 3,
        4: 1,
    }

    def __init__(self, debug=False):
        self.pwm = PWM(address=0x40, debug=debug)
        self.set_OFF()
        # hardware testing flag
        self._is_performing_test = False

    def start_hardware_test(self):
        self._is_performing_test = True
    
    def finish_hardware_test(self):
        self._is_performing_test = False

    def _set_led_brightness(self, led: int, offset: int, brightness: int):
        """Sets value for brightness for one color on one LED.

        Calls the function pwm.setPWM to set the PWM signal according to
        the input brightness.

        Typically, shouldn't be used directly. Use :obj:`setRGB` instead.

        Args:
            led (:obj:`int`): Index of specific LED (from the table above)
            offset (:obj:`int`): Offset for color
            brightness (:obj:`int8`): Intensity of brightness (between 0 and 255)

        """
        # remap vehicle LED to physical LED
        led = self.VEHICLE_TO_PHYSICAL_LED_MAPPING[led]
        self.pwm.setPWM(3 * led + offset, brightness << 4, 4095)

    def set_OFF(self, leds: Union[None, int, List[int]] = None):
        if leds is None:
            leds = range(5)
        if isinstance(leds, int):
            leds = [leds]
        # set all LEDs
        for led in leds:
            for ch in range(3):
                # sets fully off all the channels
                self._set_led_brightness(led, ch, 0)

    def set_RGB(self, led: int, color: Tuple[float, float, float], intensity: float = 1.0, is_test_cmd: bool = False):
        """Sets value for brightness for all channels of one LED

        Converts the input color brightness from [0,1] to [0,255] for all
        channels, then calls self.setLEDBrightness with the right offset
        corresponding to the color channel in the PWM signal and the color
        value as int8.

        Args:
            led (:obj:`int`): Index of specific LED (from the table above)
            color (:obj:`list` of :obj:`float`): Brightness for the three RGB channels, in interval [0,1]
            intensity (:obj:`float`): Intensity of the LED
            is_test_cmd (:obj:`bool`): whether this is a command issue by the hardware test
        """

        if not self._is_performing_test or is_test_cmd:
            self._set_led_brightness(led, self.OFFSET_RED, int(color[0] * intensity * 255))
            self._set_led_brightness(led, self.OFFSET_GREEN, int(color[1] * intensity * 255))
            self._set_led_brightness(led, self.OFFSET_BLUE, int(color[2] * intensity * 255))

    def __del__(self):
        """Destructur method.

        Turns off all the LEDs and deletes the PWM object.

        """
        if not hasattr(self, "pwm"):
            return
        self.set_OFF()
        del self.pwm
