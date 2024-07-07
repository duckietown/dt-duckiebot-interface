from abc import ABC
from enum import IntEnum
from typing import Optional


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
            resolution (:obj:`int`): number of ticks per revolution.
    """

    def __init__(self, name: str, resolution: int):
        self._name: str = name
        self._resolution: int = resolution
        self._ticks: int = 0
        self._emulation_last: Optional[int] = None
        # public
        self.emulated: bool = False
        # wheel direction
        self._direction: WheelDirection = WheelDirection.FORWARD

    @property
    def ticks(self) -> int:
        return self._ticks

    def emulate_another_sensor(self, ticks: int):
        # if we don't have a last tick or the difference is too big (more than one full revolution), reset the emulation
        if self._emulation_last is None or abs(ticks - self._emulation_last) > self._resolution:
            self._emulation_last = ticks
            return
        # compute difference in ticks
        diff: int = ticks - self._emulation_last
        # update emulation state
        self._emulation_last = ticks
        # set the new tick value
        self._ticks += diff

    def _set_ticks(self, ticks: int):
        if self.emulated:
            return
        self._ticks = ticks

    def _bump_ticks(self, _):
        if self.emulated:
            return
        self._ticks += self._direction.value

    def get_direction(self) -> WheelDirection:
        return self._direction

    def set_direction(self, direction: WheelDirection):
        self._direction = direction

    def release(self):
        pass
