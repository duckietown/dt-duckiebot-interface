import dataclasses
import numpy as np

from dt_vl53l0x import Vl53l0xAccuracyMode


@dataclasses.dataclass
class ToFAccuracy:
    mode: Vl53l0xAccuracyMode
    timing_budget: float
    max_range: float
    # the following are taken from the sensor's datasheet
    min_range: float = 0.05
    fov: float = np.deg2rad(25)

    @staticmethod
    def from_string(mode: str):
        ms = 1 / 1000
        return {
            "GOOD": ToFAccuracy(Vl53l0xAccuracyMode.GOOD, 33 * ms, 1.2),
            "BETTER": ToFAccuracy(Vl53l0xAccuracyMode.BETTER, 66 * ms, 1.2),
            "BEST": ToFAccuracy(Vl53l0xAccuracyMode.BEST, 200 * ms, 1.2),
            "LONG_RANGE": ToFAccuracy(Vl53l0xAccuracyMode.LONG_RANGE, 33 * ms, 2.0),
            "HIGH_SPEED": ToFAccuracy(Vl53l0xAccuracyMode.HIGH_SPEED, 20 * ms, 1.2),
        }[mode]
