#!/usr/bin/env python3
import os

import numpy as np

from dt_duckiematrix_protocols.world.WheelsCommand import WheelsCommand
from dt_duckiematrix_utils.socket import DuckieMatrixSocket
from dt_robot_utils import get_robot_configuration, get_robot_name


class VirtualWheelsDriver:
    """Class handling communication with virtual motors.

    """
    LEFT_MOTOR_MIN_SPEED = 0.2  #: Minimum speed for left motor
    LEFT_MOTOR_MAX_SPEED = 1.0  #: Maximum speed for left motor
    RIGHT_MOTOR_MIN_SPEED = 0.2  #: Minimum speed for right motor
    RIGHT_MOTOR_MAX_SPEED = 1.0  #: Maximum speed for right motor
    SPEED_TOLERANCE = 1.e-2  #: Speed tolerance level

    def __init__(self):
        rcfg = get_robot_configuration()
        # print out some stats
        this = self.__class__.__name__
        print(f"[{this}] Running in configuration `{rcfg.name}`, using driver `virtual`")
        print(f"[{this}] Motor #1: VIRTUAL(left)")
        print(f"[{this}] Motor #2: VIRTUAL(right)")
        # prepare zmq pipeline
        self._device: DuckieMatrixSocket = DuckieMatrixSocket.create()
        if self._device is None or not self._device.connected:
            print("[VirtualMotors]: No virtual Motor connection established.")
        else:
            print("[VirtualMotors]: Initialized.")
        # setup topic
        self._topic = os.path.join(get_robot_name(), "wheels")
        # initialize state
        self._wheels = {
            "left": 0.0,
            "right": 0.0,
        }
        self._publish()

    def set_wheels_speed(self, left: float, right: float):
        """Sets speed of motors.

        Args:
           left (:obj:`float`): speed for the left wheel, should be between -1 and 1
           right (:obj:`float`): speed for the right wheel, should be between -1 and 1

        """
        self._wheels = {
            "left": clamped_value(left, self.SPEED_TOLERANCE, self.LEFT_MOTOR_MIN_SPEED,
                                  self.LEFT_MOTOR_MAX_SPEED),
            "right": clamped_value(right, self.SPEED_TOLERANCE, self.RIGHT_MOTOR_MIN_SPEED,
                                   self.RIGHT_MOTOR_MAX_SPEED)
        }
        self._publish()

    def _publish(self):
        """
        Sends commands.
        """
        if self._device.connected:
            message = WheelsCommand(self._wheels)
            self._device.publish(self._topic, message)

    def __del__(self):
        if self._device is not None:
            print('[VirtualMotors]: Releasing...')
            self._device.release()
            print('[VirtualMotors]: Released.')
        self._device = None


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
