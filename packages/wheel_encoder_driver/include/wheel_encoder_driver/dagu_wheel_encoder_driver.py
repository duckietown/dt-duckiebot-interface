from dt_device_utils import get_device_hardware_brand, DeviceHardwareBrand

from .wheel_encoder_abs import WheelEncoderDriverAbs, WheelDirection

ROBOT_HARDWARE = get_device_hardware_brand()

if ROBOT_HARDWARE == DeviceHardwareBrand.JETSON_NANO:
    import Jetson.GPIO as GPIO

elif ROBOT_HARDWARE in [DeviceHardwareBrand.RASPBERRY_PI, DeviceHardwareBrand.RASPBERRY_PI_64]:
    import RPi.GPIO as GPIO

else:
    raise Exception("Undefined Hardware!")


class DaguWheelEncoderDriver(WheelEncoderDriverAbs):
    """Class handling communication with a wheel encoder.

    An instance of this class reads data off of a wheel encoder calls a callback function
    with the new cumulative tick number as sole argument.
    The callback is called only when the encoder fires, thus there is no constant frequency.

        Args:
            name (:obj:`str`): name of the encoder (e.g., left, right).
            resolution (:obj:`int`): number of ticks per revolution.
            ticks_gpio (:obj:`int`): Number of the pin the encoder is connected to.
            direction_gpio (:obj:`int`): Number of the pin the encoder direction signal is connected to.
            direction_inverted (:obj:`bool`): Flag to invert the direction signal.
    """

    def __init__(self, name: str, resolution: int, ticks_gpio: int, direction_gpio: int, direction_inverted: bool):
        super(DaguWheelEncoderDriver, self).__init__(name, resolution)
        self._direction_correction: int = 1 if direction_inverted else 0
        # configure GPIO mode
        GPIO.setmode(GPIO.BCM)
        # validate gpio
        if not 1 <= ticks_gpio <= 40:
            raise ValueError("The pin number must be within the range [1, 40].")
        # configure GPIO pin
        self._ticks_gpio: int = ticks_gpio
        GPIO.setup(ticks_gpio, GPIO.IN)
        # validate gpio
        if not 1 <= direction_gpio <= 40:
            raise ValueError("The pin number must be within the range [1, 40].")
        # configure GPIO pin
        self._direction_gpio: int = direction_gpio
        self._direction_edge = None
        GPIO.setup(direction_gpio, GPIO.IN)
        # add event detection
        GPIO.add_event_detect(ticks_gpio, GPIO.RISING, callback=self._bump_ticks)
        GPIO.add_event_detect(direction_gpio, GPIO.BOTH, callback=self._process_direction_edge)

    def _bump_ticks(self, _):
        if self._direction_edge is GPIO.RISING:
            self.set_direction(WheelDirection.REVERSE)
        else:
            self.set_direction(WheelDirection.FORWARD)
        # ---
        super()._bump_ticks(_)

    def _process_direction_edge(self, _):
        if GPIO.input(self._direction_gpio):
            self._direction_edge = GPIO.RISING if not self._direction_correction else GPIO.FALLING
        else:
            self._direction_edge = GPIO.FALLING if not self._direction_correction else GPIO.RISING

    def release(self):
        GPIO.remove_event_detect(self._ticks_gpio)
        GPIO.remove_event_detect(self._direction_gpio)
