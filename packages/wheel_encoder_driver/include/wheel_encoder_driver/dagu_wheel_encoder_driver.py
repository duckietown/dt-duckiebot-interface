from typing import Callable

from dt_device_utils import get_device_hardware_brand, DeviceHardwareBrand

from .wheel_encoder_abs import WheelEncoderDriverAbs

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
            gpio (:obj:`int`): Number of the pin the encoder is connected to.
    """

    def __init__(self, name: str, gpio: int):
        super(DaguWheelEncoderDriver, self).__init__(name)
        # valid gpio
        if not 1 <= gpio <= 40:
            raise ValueError("The pin number must be within the range [1, 40].")
        # configure GPIO pin
        self._gpio: int = gpio
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(gpio, GPIO.IN)
        GPIO.add_event_detect(gpio, GPIO.RISING, callback=self._bump_ticks)

    def release(self):
        GPIO.remove_event_detect(self._gpio)
