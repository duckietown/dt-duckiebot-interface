import dataclasses
from abc import ABC

import numpy as np


class ToFAccuracyMode:
    GOOD = 0        # 33 ms timing budget 1.2m range
    BETTER = 1      # 66 ms timing budget 1.2m range
    BEST = 2        # 200 ms 1.2m range
    LONG_RANGE = 3  # 33 ms timing budget 2m range
    HIGH_SPEED = 4  # 20 ms timing budget 1.2m range


@dataclasses.dataclass
class ToFAccuracy:
    mode: int
    timing_budget: float
    max_range: float
    # the following are taken from the sensor's datasheet
    min_range: float = 0.03
    fov: float = np.deg2rad(25)

    @staticmethod
    def from_string(mode: str):
        ms = 1 / 1000
        return {
            "GOOD": ToFAccuracy(ToFAccuracyMode.GOOD, 33 * ms, 1.2),
            "BETTER": ToFAccuracy(ToFAccuracyMode.BETTER, 66 * ms, 1.2),
            "BEST": ToFAccuracy(ToFAccuracyMode.BEST, 200 * ms, 1.2),
            "LONG_RANGE": ToFAccuracy(ToFAccuracyMode.LONG_RANGE, 33 * ms, 2.0),
            "HIGH_SPEED": ToFAccuracy(ToFAccuracyMode.HIGH_SPEED, 20 * ms, 1.2)
        }[mode]


class ToFDriverAbs(ABC):

    def __init__(self, name: str, accuracy: ToFAccuracy):
        self._name = name
        self._accuracy = accuracy

    def setup(self, *args, **kwargs):
        raise NotImplementedError("ToFDriverAbs is an abstract class, this method should be "
                                  "implemented by the child class.")

    def start(self):
        raise NotImplementedError("ToFDriverAbs is an abstract class, this method should be "
                                  "implemented by the child class.")

    def get_distance(self) -> float:
        raise NotImplementedError("ToFDriverAbs is an abstract class, this method should be "
                                  "implemented by the child class.")

    def stop(self):
        raise NotImplementedError("ToFDriverAbs is an abstract class, this method should be "
                                  "implemented by the child class.")

    def release(self):
        raise NotImplementedError("ToFDriverAbs is an abstract class, this method should be "
                                  "implemented by the child class.")
