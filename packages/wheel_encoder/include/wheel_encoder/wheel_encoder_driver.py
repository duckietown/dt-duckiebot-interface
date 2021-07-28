#!/usr/bin/env python3

from enum import IntEnum

from dt_device_utils import get_device_hardware_brand, DeviceHardwareBrand
ROBOT_HARDWARE = get_device_hardware_brand()

if ROBOT_HARDWARE == DeviceHardwareBrand.JETSON_NANO:
    import Jetson.GPIO as GPIO

elif ROBOT_HARDWARE == DeviceHardwareBrand.RASPBERRY_PI:
    import RPi.GPIO as GPIO

else:
    raise Exception("Undefined Hardware!")


class WheelDirection(IntEnum):
    FORWARD = 1
    REVERSE = -1


class WheelEncoderDriver:
    """Class handling communication with a wheel encoder.

    An instance of this class reads data off of a wheel encoder calls a callback function
    with the new cumulative tick number as sole argument.
    The callback is called only when the encoder fires, thus there is no constant frequency.

        Args:
            gpio_pin (:obj:`int`): ID of the pin the encoder is connected to.
            callback (:obj:`callable`): callback function to receive new (unique) readings.
    """

    def __init__(self, gpio_pin, callback):
        # valid gpio_pin
        if not 1 <= gpio_pin <= 40:
            raise ValueError('The pin number must be within the range [1, 40].')
        # validate callback
        if not callable(callback):
            raise ValueError('The callback object must be a callable object')
        # configure GPIO pin
        self._gpio_pin = gpio_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(gpio_pin, GPIO.IN)
        GPIO.add_event_detect(gpio_pin, GPIO.RISING, callback=self._cb)
        # ---
        self._callback = callback
        self._ticks = 0
        # wheel direction
        self._direction = WheelDirection.FORWARD

    def get_direction(self) -> WheelDirection:
        return self._direction

    def set_direction(self, direction: WheelDirection):
        self._direction = direction

    def _cb(self, _):
        self._ticks += self._direction.value
        self._callback(self._ticks)

    def shutdown(self):
        GPIO.remove_event_detect(self._gpio_pin)
