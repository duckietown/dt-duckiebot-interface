from abc import ABC
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
    """

    def __init__(self, name: str):
        self._name: str = name
        self._ticks: int = 0
        # wheel direction
        self._direction: WheelDirection = WheelDirection.FORWARD

    @property
    def ticks(self) -> int:
        return self._ticks

    def _set_ticks(self, ticks: int):
        self._ticks = ticks

    def _bump_ticks(self, _):
        self._ticks += self._direction.value

    def get_direction(self) -> WheelDirection:
        return self._direction

    def set_direction(self, direction: WheelDirection):
        self._direction = direction

    def release(self):
        pass