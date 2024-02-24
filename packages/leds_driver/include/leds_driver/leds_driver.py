from typing import List

from Adafruit_PWM_Servo_Driver import PWM

from .leds_driver_abs import LEDsDriverAbs

uint8 = int
uint12 = int


class PWMLEDsDriver(LEDsDriverAbs):
    """
    Implement control of PWM RGB LEDs.
    """

    # Class-specific constants
    CHANNEL_RED = 0  # offset address for the red color
    CHANNEL_GREEN = 1  # offset address for the green color
    CHANNEL_BLUE = 2  # offset address for the blue color
    PWM_PERIOD = 4095
    NUM_PWM_CHANNELS = 15

    def __init__(self, debug=False):
        self.pwm = PWM(address=0x40, debug=debug)
        self.all_off()

    def __set_raw(self, pwm: int, value: uint12):
        """
        The method PWM.setPWM has the following prototype:

            PWM.setPWM(channel, on, off)

        The chip clock counts up from 0 to 4095, over and over.

            "on" tells the pin at what number during the count-up to turn ON.
            "off" tells the pin at what number during the count-up to turn OFF.
            "channel" is the pin number in [0, 15].

        For example, if on = 0 and off = 10, then channel with stay on from 0 to 9,
        and then when counter hits 10 the channel will turn off and stay off from 10 to 4095.
        Then the cycle repeats.

        :param pwm:     the pin number
        :param value:   the intensity value in [0, 4095]
        :return:
        """
        self.pwm.setPWM(pwm, value, self.PWM_PERIOD)

    def set_channel_intensity(self, led: int, channel: int, intensity: uint8):
        """
        Sets value for brightness for a single channel out of the three making up an RGB LED.

        Typically, this shouldn't be used directly. Use :obj:`set_rgb` instead.

        Args:
            led (:obj:`int`): Index of specific LED (from the table above)
            channel (:obj:`int`): Channel number from [0, 1, 2] corresponding to [R, G, B]
            intensity (:obj:`int8`): Intensity of brightness (between 0 and 255)

        """
        self.pwm.setPWM(3 * led + channel, intensity << 4, self.PWM_PERIOD)

    def set_rgb(self, led: int, color: List[uint8]):
        """
        Sets value for brightness for all channels of one RGB LED.

        Args:
            led (:obj:`int`): Index of specific LED (from the table above)
            color (:obj:`list` of :obj:`float`): Brightness for the three RGB channels, in interval [0,1]
        """

        self.set_channel_intensity(led, self.CHANNEL_RED, int(color[0]))
        self.set_channel_intensity(led, self.CHANNEL_GREEN, int(color[1]))
        self.set_channel_intensity(led, self.CHANNEL_BLUE, int(color[2]))

    def release(self):
        """Destructur method.

        Turns off all the LEDs and deletes the PWM object.

        """
        self.all_off()
        del self.pwm
