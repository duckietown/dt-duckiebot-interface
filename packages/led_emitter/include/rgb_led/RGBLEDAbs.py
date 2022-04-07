from abc import ABC, abstractmethod


class RGBLEDAbs(ABC):
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

    @abstractmethod
    def setRGB(self, led, color):
        """Sets value for brightness for all channels of one LED

            Converts the input color brightness from [0,1] to [0,255] for all
            channels, then calls self.setLEDBrightness with the right offset
            corresponding to the color channel in the PWM signal and the color
            value as int8.

            Args:
                led (:obj:`int`): Index of specific LED (from the table above)    
                color (:obj:`list` of :obj:`float`): Brightness for the three
                                                     RGB channels, in interval [0,1]
        """
        raise NotImplementedError("Reached abstract method `setRGB()`")

    @abstractmethod
    def __del__(self):
        """Destructur method.

            Turns off all the LEDs and deletes the PWM object.

        """
        raise NotImplementedError("Reached abstract method `__del__()`")
