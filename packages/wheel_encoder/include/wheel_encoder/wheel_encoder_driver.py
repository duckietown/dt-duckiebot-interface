#!/usr/bin/env python3

from enum import IntEnum, auto

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


class WheelSide(IntEnum):
    LEFT = auto()
    RIGHT = auto()


class WheelEncoderDriver:
    """Class handling communication with a wheel encoder.

    An instance of this class reads data off of a wheel encoder calls a callback function
    with the new cumulative tick number as sole argument.
    The callback is called only when the encoder fires, thus there is no constant frequency.

        Args:
            gpio_pins (:obj:`int`): IDs of the pins the encoder is connected to.
            callback (:obj:`callable`): callback function to receive new (unique) readings.
    """

    def __init__(self, gpio_pins, configuration: WheelSide, callback):
        # validate callback
        if not callable(callback):
            raise ValueError('The callback object must be a callable object')

        self._configuration: WheelSide = configuration

        # configure GPIO pins
        GPIO.setmode(GPIO.BCM)

        # validate gpio_pin(s)
        ast_msg_num = "{} based robots must have exactly {} gpio_pin(s) for a wheel encoder!"
        ast_msg_range = "The pin number must be within the range [1, 40]."
        if ROBOT_HARDWARE == DeviceHardwareBrand.RASPBERRY_PI:
            assert len(gpio_pins) == 1, ast_msg_num.format(ROBOT_HARDWARE.name, 1)
            pin = gpio_pins[0]
            assert 1 <= pin <= 40, ast_msg_range
            GPIO.setup(pin, GPIO.IN)
            self._gpio_pin1 = pin
        elif ROBOT_HARDWARE == DeviceHardwareBrand.JETSON_NANO:
            assert len(gpio_pins) == 2, ast_msg_num.format(ROBOT_HARDWARE.name, 2)
            for pin in gpio_pins:
                assert 1 <= pin <= 40, ast_msg_range
                GPIO.setup(pin, GPIO.IN)
            self._gpio_pin1, self._gpio_pin2 = gpio_pins
        # Both RPi and Jetson have _gpio_pin1, only Jetson has _gpio_pin2

        GPIO.add_event_detect(self._gpio_pin1, GPIO.RISING, callback=self._cb)
        # ---
        self._callback = callback
        self._ticks = 0
        # wheel direction
        self._direction = WheelDirection.FORWARD

    def set_direction(self, direction: WheelDirection):
        self._direction = direction

    def _cb(self, _):
        if ROBOT_HARDWARE == DeviceHardwareBrand.JETSON_NANO:
            phase = GPIO.input(self._gpio_pin2)
            if phase is GPIO.LOW:
                rot_clockwise = False  # counter-clockwise
            elif phase is GPIO.HIGH:
                rot_clockwise = True   # clockwise
            else:
                raise ValueError(f"Wheel encoder phase UNKNOWN! Value: {phase}")
            
            if self._configuration == WheelSide.RIGHT:
                direction = WheelDirection.FORWARD if rot_clockwise else WheelDirection.REVERSE
            else:  # left wheel
                direction = WheelDirection.REVERSE if rot_clockwise else WheelDirection.FORWARD
            self.set_direction(direction)

        self._ticks += self._direction.value
        self._callback(self._ticks)

    def shutdown(self):
        GPIO.remove_event_detect(self._gpio_pin1)
        GPIO.cleanup()
