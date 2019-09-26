#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM


class RGB_LED():
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

    # Class-specific constants
    OFFSET_RED = 0    #: Offset address for the red color
    OFFSET_GREEN = 1  #: Offset address for the green color
    OFFSET_BLUE = 2   #: Offset address for the blue color

    def __init__(self, debug=False):
        self.pwm = PWM(address=0x40, debug=debug)
        for i in range(15):
            # Sets fully off all the pins
            self.pwm.setPWM(i, 0, 4095)

    def setLEDBrightness(self, led, offset, brightness):
        """Sets value for brightness for one color on one LED.

            Calls the function pwm.setPWM to set the PWM signal according to
            the input brightness.

            Typically shouldn't be used directly. Use :obj:`setRGB` instead.

            Args:
                led (:obj:`int`): Index of specific LED (from the table above)
                offset (:obj:`int`): Offset for color
                brightness (:obj:`int8`): Intensity of brightness (between 0 and 255)

        """
        self.pwm.setPWM(3 * led + offset, brightness << 4, 4095)

    def setRGB(self, led, color):
        """Sets value for brightness for all channels of one LED

            Converts the input color brightness from [0,1] to [0,255] for all
            channels, then calls self.setLEDBrightness with the right offset
            corresponding to the color channel in the PWM signal and the color
            value as int8.

            Args:
                led (:obj:`int`): Index of specific LED (from the table above)    
                color (:obj:`list` of :obj:`float`): Brightness for the three RGB channels, in interval [0,1]
        """

        self.setLEDBrightness(led, self.OFFSET_RED, int(color[0] * 255))
        self.setLEDBrightness(led, self.OFFSET_GREEN, int(color[1] * 255))
        self.setLEDBrightness(led, self.OFFSET_BLUE, int(color[2] * 255))

    def __del__(self):
        """Destructur method.

            Turns off all the LEDs and deletes the PWM object.

        """
        for i in range(15):
            # Sets fully off all channels of all the LEDs (3 channles * 5 LEDs)
            self.pwm.setPWM(i, 0, 4095)
        del self.pwm
