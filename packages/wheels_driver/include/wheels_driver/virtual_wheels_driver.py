#!/usr/bin/env python3

import numpy as np

from wheels_driver.wheels_driver_abs import WheelsDriverAbs, WheelPWMConfiguration

float1 = float


class VirtualWheelsDriver(WheelsDriverAbs):
    """Class handling communication with virtual motors.

    """

    def __init__(self, left_config: WheelPWMConfiguration, right_config: WheelPWMConfiguration):
        super(VirtualWheelsDriver, self).__init__(left_config, right_config)
        # ---
        self._left_pwm: float = 0
        self._right_pwm: float = 0

    def set_wheels_speed(self, left: float, right: float):
        # clamp the values
        # - left
        left_config = self.left_config
        self._left_pwm = clamped_value(
            left, left_config.deadzone, left_config.pwm_min / 255., left_config.pwm_max / 255.
        )
        # - right
        right_config = self.right_config
        self._right_pwm = clamped_value(
            right, right_config.deadzone, right_config.pwm_min / 255., right_config.pwm_max / 255.
        )

    @property
    def left_pwm(self) -> float1:
        return self._left_pwm
    
    @property
    def right_pwm(self) -> float1:
        return self._right_pwm


def clamped_value(v, deadzone, min_v, max_v) -> float:
    """Transforms the requested speed into a clamped float number.

        Args:
            v (:obj:`float`): requested speed, should be between -1 and 1.
            deadzone (:obj:`float`): deadzone speed as float
            min_v (:obj:`float`): minimum speed as float
            max_v (:obj:`float`): maximum speed as float
    """
    value = 0
    if np.abs(v) > deadzone:
        value = np.sign(v) * (np.abs(v) * (max_v - min_v) + min_v)
    return min(value, max_v)
