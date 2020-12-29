from abc import abstractmethod, ABC
from collections import namedtuple
from enum import IntEnum

from Adafruit_PWM_Servo_Driver import PWM
from typing import Dict

from .constants import LOW, HIGH
from .gpio import LOW as GPIO_LOW, HIGH as GPIO_HIGH
from .pwm import LOW as PWM_LOW, HIGH as PWM_HIGH

from dt_device_utils import get_device_hardware_brand, DeviceHardwareBrand
ROBOT_HARDWARE = get_device_hardware_brand()

if ROBOT_HARDWARE == DeviceHardwareBrand.RASPBERRY_PI:
    import RPi.GPIO as GPIO
elif ROBOT_HARDWARE == DeviceHardwareBrand.JETSON_NANO:
    import Jetson.GPIO as GPIO
else:
    raise ValueError(f"Hardware `{ROBOT_HARDWARE.name}` not supported.")


class MotorDirection(IntEnum):
    RELEASE = 0
    FORWARD = 1
    BACKWARD = -1


class MotorDirectionControl(IntEnum):
    PWM = 0
    GPIO = 1


class AbsMotorDirectionController(ABC):

    def __init__(self, in1_pin: int, in2_pin: int, *args, **kwargs):
        self._in1_pin = in1_pin
        self._in2_pin = in2_pin
        self.setup()

    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def set(self, direction: MotorDirection):
        pass


class PWMMotorDirectionController(AbsMotorDirectionController):
    _DIRECTION_TO_SIGNALS = {
        MotorDirection.RELEASE: (LOW, HIGH),
        MotorDirection.FORWARD: (HIGH, LOW),
        MotorDirection.BACKWARD: (LOW, HIGH)
    }
    _PWM_VALUES = {
        LOW: (PWM_LOW, PWM_HIGH),
        HIGH: (PWM_HIGH, PWM_LOW),
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if not isinstance(kwargs.get('pwm', None), PWM):
            raise ValueError("You cannot instantiate `PWMMotorDirectionController` "
                             "without passing a `PWM` object.")
        self._pwm = kwargs['pwm']

    def setup(self):
        pass

    def set(self, direction: MotorDirection):
        in1_signal, in2_signal = self._DIRECTION_TO_SIGNALS[direction]
        in1_value, in2_value = self._PWM_VALUES[in1_signal], self._PWM_VALUES[in2_signal]
        self._pwm.setPWM(self._in1_pin, *in1_value)
        self._pwm.setPWM(self._in2_pin, *in2_value)


class GPIOMotorDirectionController(AbsMotorDirectionController):
    _DIRECTION_TO_SIGNALS = {
        MotorDirection.RELEASE: (HIGH, HIGH),
        MotorDirection.FORWARD: (HIGH, LOW),
        MotorDirection.BACKWARD: (LOW, HIGH)
    }
    _GPIO_VALUES = {
        LOW: GPIO_LOW,
        HIGH: GPIO_HIGH
    }

    def setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._in1_pin, GPIO.OUT)
        GPIO.setup(self._in2_pin, GPIO.OUT)

    def set(self, direction: MotorDirection):
        in1_signal, in2_signal = self._DIRECTION_TO_SIGNALS[direction]
        in1_value, in2_value = self._GPIO_VALUES[in1_signal], self._GPIO_VALUES[in2_signal]
        GPIO.output(self._in1_pin, in1_value)
        GPIO.output(self._in2_pin, in2_value)


MotorPins = namedtuple('MotorPins', 'in1 in2 pwm')


class Motor:
    _K = 16
    _CONTROLLER = {
        MotorDirectionControl.PWM: PWMMotorDirectionController,
        MotorDirectionControl.GPIO: GPIOMotorDirectionController,
    }

    def __init__(self, pwm: PWM, in1_pin: int, in2_pin: int, pwm_pin: int, control: MotorDirectionControl):
        self._pwn = pwm
        self._pwm_pin = pwm_pin
        self._controller = self._CONTROLLER[control](in1_pin, in2_pin)

    def set(self, direction: MotorDirection, speed: int):
        speed = max(0, min(speed, 255))
        self._controller.set(direction)
        self._pwn.setPWM(self._pwm_pin, 0, speed * self._K)







class AbsHAT(ABC):

    def __init__(self, address=0x60, frequency=1600):
        # default I2C address of the HAT
        self._i2caddr = address
        # default @1600Hz PWM frequency
        self._frequency = frequency
        # configure PWM
        self._pwm = PWM(self._i2caddr, debug=False)
        self._pwm.setPWMFreq(self._frequency)

    @abstractmethod
    def get_motor(self, num: int) -> Motor:
        pass


class HATv1(AbsHAT):
    _MOTOR_NUM_TO_PINS: Dict[int, MotorPins] = {
        0: MotorPins(10, 9, 8),
        1: MotorPins(11, 12, 13),
        2: MotorPins(4, 3, 2),
        3: MotorPins(5, 6, 7),
    }

    def get_motor(self, num: int) -> Motor:
        if num not in self._MOTOR_NUM_TO_PINS:
            raise ValueError(f'Motor num `{num}` not supported. '
                             f'Possible choices are `{self._MOTOR_NUM_TO_PINS.keys()}`.')
        pins = self._MOTOR_NUM_TO_PINS[num]
        return Motor(pins.in1, pins.in2, pins.pwm)





