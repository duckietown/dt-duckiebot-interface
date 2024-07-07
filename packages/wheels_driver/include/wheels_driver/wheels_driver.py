#!/usr/bin/env python3

from math import fabs, floor

import hat_driver
from dt_robot_utils import get_robot_configuration
from wheels_driver.wheels_driver_abs import WheelsDriverAbs, WheelPWMConfiguration

MotorDirection = hat_driver.MotorDirection
uint8 = int
float1 = float


class DaguWheelsDriver(WheelsDriverAbs):
    """Class handling communication with motors."""

    def __init__(self, left_config: WheelPWMConfiguration, right_config: WheelPWMConfiguration):
        super(DaguWheelsDriver, self).__init__(left_config, right_config)
        rcfg = get_robot_configuration()
        DTHAT = hat_driver.from_env()
        self.hat = DTHAT()
        self.leftMotor = self.hat.get_motor(1, "left")
        self.rightMotor = self.hat.get_motor(2, "right")
        # print out some stats
        this = self.__class__.__name__
        print(f"[{this}] Running in configuration `{rcfg.name}`, using driver `{DTHAT.__name__}`")
        print(f"[{this}] Motor #1: {self.leftMotor}")
        print(f"[{this}] Motor #2: {self.rightMotor}")
        # pwm (executed, in the range [0, 1])
        self._executed_left: float1 = 0.0
        self._executed_right: float1 = 0.0
        # ---
        self.set_wheels_speed(0, 0)

    @property
    def left_pwm(self):
        return self._executed_left

    @property
    def right_pwm(self):
        return self._executed_right

    def set_wheels_speed(self, left: float, right: float):
        """Sends commands to the microcontroller.

        Updates the current PWM signals (left and right) according to the
        linear velocities of the motors. The requested speed gets
        tresholded.

        Args:
           left (:obj:`float`): speed for the left wheel, should be between -1 and 1
           right (:obj:`float`): speed for the right wheel, should be between -1 and 1

        """
        pwml: uint8 = self._pwm_value(left, self.left_config)
        pwmr: uint8 = self._pwm_value(right, self.right_config)
        leftMotorMode = MotorDirection.RELEASE
        rightMotorMode = MotorDirection.RELEASE

        # figure out the directions
        if fabs(left) < self.left_config.deadzone:
            pwml = 0
        elif left > 0:
            leftMotorMode = MotorDirection.FORWARD
        elif left < 0:
            leftMotorMode = MotorDirection.BACKWARD

        if fabs(right) < self.right_config.deadzone:
            pwmr = 0
        elif right > 0:
            rightMotorMode = MotorDirection.FORWARD
        elif right < 0:
            rightMotorMode = MotorDirection.BACKWARD

        # executed pwm values are floats in [-1, 1] encoding both speed and direction
        self._executed_left = (pwml * leftMotorMode.value) / 255.
        self._executed_right = (pwmr * rightMotorMode.value) / 255.

        # set PWM signals
        if not self.pretend:
            self.leftMotor.set(leftMotorMode, pwml)
            self.rightMotor.set(rightMotorMode, pwmr)

    @staticmethod
    def _pwm_value(v: float, wheel_config: WheelPWMConfiguration) -> uint8:
        """Transforms the requested speed into an int8 number.

        Args:
            v (:obj:`float`): requested speed of the wheel in [-1, 1]
            wheel_config (:obj:`WheelPWMConfiguration`): the PWM configuration of the wheel
        """
        pwm: uint8 = 0
        if fabs(v) > wheel_config.deadzone:
            pwm = int(floor(fabs(v) * (wheel_config.pwm_max - wheel_config.pwm_min) + wheel_config.pwm_min))
        return min(pwm, wheel_config.pwm_max)

    def __del__(self):
        """Destructor method.

        Releases the motors and deletes tho object.
        """
        self.leftMotor.set(MotorDirection.RELEASE)
        self.rightMotor.set(MotorDirection.RELEASE)
        del self.hat
