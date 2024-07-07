import dataclasses
from abc import ABC, abstractmethod

uint8 = int
float01 = float


@dataclasses.dataclass
class WheelPWMConfiguration:
    """
    Fields:
        pwm_min: minimum speed as uint8 (i.e., in [0, 255])
        pwm_max: maximum speed as uint8 (i.e., in [0, 255])
        deadzone: deadzone radius as float in [0, 1)
    """
    pwm_min: uint8 = 60
    pwm_max: uint8 = 255
    deadzone: float01 = 0.2


class WheelsDriverAbs(ABC):

    def __init__(self, left_config: WheelPWMConfiguration, right_config: WheelPWMConfiguration):
        self.left_config = left_config
        self.right_config = right_config
        self.pretend: bool = False

    @abstractmethod
    def set_wheels_speed(self, left: float, right: float):
        """Sets speed of motors.

        Args:
           left (:obj:`float`): speed for the left wheel, should be between -1 and 1
           right (:obj:`float`): speed for the right wheel, should be between -1 and 1

        """
        pass

    @property
    @abstractmethod
    def left_pwm(self) -> float:
        pass

    @property
    @abstractmethod
    def right_pwm(self) -> float:
        pass
