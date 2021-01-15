from typing import Dict
from abc import abstractmethod, ABC

from Adafruit_PWM_Servo_Driver import PWM

from .motor import Motor, MotorPins, MotorDirectionControl


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
    def get_motor(self, num: int, name: str) -> Motor:
        pass


class HATv1(AbsHAT):
    _MOTOR_NUM_TO_PINS: Dict[int, MotorPins] = {
        1: MotorPins(10, 9, 8, MotorDirectionControl.PWM),
        2: MotorPins(11, 12, 13, MotorDirectionControl.PWM),
        3: MotorPins(4, 3, 2, MotorDirectionControl.PWM),
        4: MotorPins(5, 6, 7, MotorDirectionControl.PWM),
    }

    def get_motor(self, num: int, name: str) -> Motor:
        if num not in self._MOTOR_NUM_TO_PINS:
            raise ValueError(f'Motor num `{num}` not supported. '
                             f'Possible choices are `{self._MOTOR_NUM_TO_PINS.keys()}`.')
        pins = self._MOTOR_NUM_TO_PINS[num]
        return Motor(name, self._pwm, pins.in1, pins.in2, pins.pwm, control=pins.control)


class HATv2(HATv1):
    pass


class HATv3(HATv2):
    _MOTOR_NUM_TO_PINS: Dict[int, MotorPins] = {
        1: MotorPins(10, 9, 8, MotorDirectionControl.PWM),
        2: MotorPins(33, 31, 13, MotorDirectionControl.GPIO)
    }
