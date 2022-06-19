#!/usr/bin/env python3

from typing import Optional

import numpy as np

from dt_duckiematrix_protocols import Matrix
from dt_duckiematrix_protocols.robot.features.motion import DifferentialDrive
from dt_duckiematrix_protocols.robot.robots import DifferentialDriveRobot
from dt_duckiematrix_utils.ros import DuckiematrixLinkDescription, \
    on_duckiematrix_connection_request
from dt_robot_utils import get_robot_configuration


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
        # initialize state
        self._wheels = {
            "left": 0.0,
            "right": 0.0,
        }
        # print out some stats
        this = self.__class__.__name__
        print(f"[{this}] Running in configuration `{rcfg.name}`, using driver `virtual`")
        print(f"[{this}] Motor #1: VIRTUAL(left)")
        print(f"[{this}] Motor #2: VIRTUAL(right)")
        # register connection setup function
        self._matrix: Optional[Matrix] = None
        self._device: Optional[DifferentialDrive] = None
        # register connection setup function
        print(f"[VirtualMotors]: Waiting for connection request...")
        self._connection_request: Optional[DuckiematrixLinkDescription] = None
        on_duckiematrix_connection_request(self.on_connection_request)

    def on_connection_request(self, link: DuckiematrixLinkDescription):
        print(f"[VirtualMotors]: Received request to connect to Duckiematrix '{link.matrix}'.")
        # store new connection request
        self._connection_request = link
        # switch over to the new connection
        self.release()
        self.setup()

    def setup(self):
        if self._connection_request is None:
            return
        # ---
        link = self._connection_request
        configuration = get_robot_configuration()
        # prepare zmq pipeline
        self._matrix: Matrix = Matrix(link.uri, auto_commit=True)
        robot: DifferentialDriveRobot = self._matrix.robots.create(configuration.name, link.entity)
        self._device: DifferentialDrive = robot.drive
        self._publish()
        print(f"[VirtualMotors]: Initialized.")

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
        if self._device is None:
            return
        # ---
        self._device(self._wheels["left"], self._wheels["right"])

    def release(self):
        if self._device is not None:
            print('[VirtualMotors]: Releasing...')
            self._device(0.0, 0.0)
            print('[VirtualMotors]: Released.')
        self._device = None

    def __del__(self):
        self.release()


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
