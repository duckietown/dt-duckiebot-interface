import dataclasses
from abc import ABC
from enum import IntEnum

import numpy as np


class ToFAccuracyModeVL53L0X:
    GOOD = 0        # 33 ms timing budget 1.2m range
    BETTER = 1      # 66 ms timing budget 1.2m range
    BEST = 2        # 200 ms 1.2m range
    LONG_RANGE = 3  # 33 ms timing budget 2m range
    HIGH_SPEED = 4  # 20 ms timing budget 1.2m range

class ToFAccuracyModeVL53L1X(IntEnum):
    """Accuracy modes for the VL53L1X sensor.
    1=short (up to 136cm) , 2=long (up to 360cm)
    """
    SHORT_RANGE = 1
    LONG_RANGE = 2


@dataclasses.dataclass
class ToFAccuracy:
    mode: int
    timing_budget: float
    max_range: float
    # the following are taken from the sensor's datasheet
    min_range: float = 0.03
    fov: float = np.deg2rad(25)
    
    @staticmethod
    def from_string(mode: str, sensor_model: str = "VL53L0X"):
        ms = 1 / 1000
    
        if sensor_model == "VL53L0X":
            return {
                "GOOD": ToFAccuracy(ToFAccuracyModeVL53L0X.GOOD, 33 * ms, 1.2),
                "BETTER": ToFAccuracy(ToFAccuracyModeVL53L0X.BETTER, 66 * ms, 1.2),
                "BEST": ToFAccuracy(ToFAccuracyModeVL53L0X.BEST, 200 * ms, 1.2),
                "LONG_RANGE": ToFAccuracy(ToFAccuracyModeVL53L0X.LONG_RANGE, 33 * ms, 2.0),
                "HIGH_SPEED": ToFAccuracy(ToFAccuracyModeVL53L0X.HIGH_SPEED, 20 * ms, 1.2)
            }[mode]
        elif sensor_model == "VL53L1X":
            return {
                "SHORT_RANGE": ToFAccuracy(
                    mode=ToFAccuracyModeVL53L1X.SHORT_RANGE,
                    timing_budget=20 * ms,
                    max_range=1.36,
                    min_range=0.04,
                    fov=np.deg2rad(27)
                    ),
                "LONG_RANGE": ToFAccuracy(
                    mode=ToFAccuracyModeVL53L1X.LONG_RANGE,
                    timing_budget=140 * ms,
                    max_range=3.6,
                    min_range=0.04,
                    fov=np.deg2rad(27)
                    )
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
        """Return the range from the sensor in millimeters.

        Raises:
            NotImplementedError: If the method is not implemented by the child class.

        Returns:
            float: The sensor's range reading in millimeters.
        """
        raise NotImplementedError("ToFDriverAbs is an abstract class, this method should be "
                                  "implemented by the child class.")

    def stop(self):
        raise NotImplementedError("ToFDriverAbs is an abstract class, this method should be "
                                  "implemented by the child class.")

    def release(self):
        raise NotImplementedError("ToFDriverAbs is an abstract class, this method should be "
                                  "implemented by the child class.")
