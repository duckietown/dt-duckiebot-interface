from abc import ABC, abstractmethod
from typing import List

uint8 = int


class LEDsDriverAbs(ABC):
    """
    Implement control of the RGB LEDs.

    Each LED is indexed by a number:

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
    red, green, and blue channels to a value between 0 and 1. The communication
    with the hardware is abstracted through the :obj:`set_rgb` method. By
    using it, you can set directly set the desired color to any LED.

    """

    @abstractmethod
    def set_channel_intensity(self, led: int, channel: int, intensity: float):
        """
        Sets value for brightness for a single channel out of the three making up an RGB LED.

        Typically, this shouldn't be used directly. Use :obj:`set_rgb` instead.

        Args:
            led (:obj:`int`): Index of specific LED (from the table above)
            channel (:obj:`int`): Channel number from [0, 1, 2] corresponding to [R, G, B]
            intensity (:obj:`float`): Intensity of brightness (between 0 and 1)

        """
        pass

    @abstractmethod
    def set_rgb(self, led: int, color: List[float]):
        """
        Sets value for brightness for all channels of one RGB LED.

        Args:
            led (:obj:`int`): Index of specific LED (from the table above)
            color (:obj:`list` of :obj:`float`): Brightness for the three RGB channels, in interval [0,1]
        """
        pass

    @abstractmethod
    def release(self):
        """
        Method to call when the LEDs are released.
        """
        pass

    def all_on(self):
        for led in range(5):
            for channel in range(3):
                self.set_channel_intensity(led, channel, 255)

    def all_off(self):
        for led in range(5):
            for channel in range(3):
                self.set_channel_intensity(led, channel, 0)

    def __del__(self):
        """Destructur method.

        Turns off all the LEDs and deletes the PWM object.

        """
        self.release()
