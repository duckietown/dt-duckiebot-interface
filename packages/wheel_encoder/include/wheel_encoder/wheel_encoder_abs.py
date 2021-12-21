from abc import ABC
from typing import Callable
from enum import IntEnum


class WheelDirection(IntEnum):
    FORWARD = 1
    REVERSE = -1


class WheelEncoderDriverAbs(ABC):
    """Class handling communication with a wheel encoder.

    An instance of this class reads data off of a wheel encoder calls a callback function
    with the new cumulative tick number as sole argument.
    The callback is called only when the encoder fires, thus there is no constant frequency.

        Args:
            name (:obj:`str`): name of the encoder (e.g., left, right).
            callback (:obj:`callable`): callback function to receive new (unique) readings.
    """

    def __init__(self, name: str, callback: Callable):
        # validate callback
        if not callable(callback):
            raise ValueError('The callback object must be a callable object')
        # ---
        self._name = name
        self._callback = callback
        self._ticks = 0
        # wheel direction
        self._direction = WheelDirection.FORWARD

    def get_direction(self) -> WheelDirection:
        return self._direction

    def set_direction(self, direction: WheelDirection):
        self._direction = direction

    def release(self):
        pass
