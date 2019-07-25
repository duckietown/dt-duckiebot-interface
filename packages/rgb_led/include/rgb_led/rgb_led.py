#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM  # @UnresolvedImport


class RGB_LED():
    """Writes commands for LEDs to the microcontroller

    """
    #
    #     LED_REAR_LEFT   = 0
    #     LED_REAR_RIGH   = 3
    #     LED_TOP         = 6
    #     LED_FRONT_LEFT  = 9
    #     LED_FRONT_RIGHT = 12

    OFFSET_RED = 0
    OFFSET_GREEN = 1
    OFFSET_BLUE = 2

    def __init__(self, debug=False):
        self.pwm = PWM(address=0x40, debug=debug)
        for i in range(15):
            # Sets fully off all the pins
            self.pwm.setPWM(i, 0, 4095)

    def setLEDBrightness(self, led, offset, brightness):
        """Sets value for brightness for one color on one LED.

        Calls the function pwm.setPWM to set the PWM signal according to the
        input brightness.

        Args:
            led (int): Port of specific LED
            offset (int): Offset for color
            brightness (int8): Intensity of brightness

        """
        self.pwm.setPWM(3 * led + offset, brightness << 4, 4095)

    def setRGB(self, led, color):
        """Sets value for brightness for all channels of one LED

        Converts the input color brightness from [0,1] to [0,255] for all
        channels, then calls self.setLEDBrightness with the right offset
        corresponding to the color channel in the PWM signal and the color
        value as int8

        Args:
            led (int): Port of specific LED
            color (list): Brightness of all RGB channels, in interval [0,1]
        """
        # Maps values in [0,1]^3 to [0,255]^3
        colorint8 = map(lambda f: int(f * 255), color)

        self.setLEDBrightness(led, self.OFFSET_RED, colorint8[0])
        self.setLEDBrightness(led, self.OFFSET_GREEN, colorint8[1])
        self.setLEDBrightness(led, self.OFFSET_BLUE, colorint8[2])

    def __del__(self):
        """Destructur method.

        Turns off all the LEDs and deletes the PWM object.

        """
        for i in range(15):
            # Sets fully off all channels of all the LEDs (3 channles * 5 LEDs)
            self.pwm.setPWM(i, 0, 4095)
        del self.pwm
