import dataclasses
from abc import ABC
from enum import IntEnum
from typing import Optional, Tuple

import numpy as np


class ToFAccuracyModeVL53L0X:
    GOOD = 0        # 33 ms timing budget 1.2m range
    BETTER = 1      # 66 ms timing budget 1.2m range
    BEST = 2        # 200 ms 1.2m range
    LONG_RANGE = 3  # 33 ms timing budget 2m range
    HIGH_SPEED = 4  # 20 ms timing budget 1.2m range

class ToFAccuracyModeVL53L1X(IntEnum):
    """Window of return times the chip accepts a peak in, independent of the timing
    budget. Short refuses late returns, which limits its reach but makes it the mode
    that copes best with bright ambient light. Long accepts them and reaches further.
    """
    SHORT_RANGE = 1
    LONG_RANGE = 2


# timing budgets the VL53L1X accepts, in milliseconds, per distance mode
VL53L1X_TIMING_BUDGETS_MS: dict = {
    ToFAccuracyModeVL53L1X.SHORT_RANGE: (15, 20, 33, 50, 100, 200, 500),
    ToFAccuracyModeVL53L1X.LONG_RANGE: (20, 33, 50, 100, 200, 500),
}


@dataclasses.dataclass
class ToFAccuracy:
    """``timing_budget`` is how long the chip integrates photons for one measurement,
    ``inter_measurement_period`` is how often it starts a new one. The period is what
    caps the measurement rate and must be at least as long as the budget.
    """
    mode: int
    timing_budget: float
    # readings beyond this are discarded. Not a capability figure: the distance actually
    # reached falls with ambient light and low target reflectance
    max_range: float
    # the following are taken from the sensor's datasheet
    min_range: float = 0.03
    fov: float = np.deg2rad(25)
    # seconds; None means "leave whatever the chip powered up with"
    inter_measurement_period: Optional[float] = None

    @property
    def timing_budget_ms(self) -> int:
        return int(round(self.timing_budget * 1000))

    @property
    def inter_measurement_period_ms(self) -> Optional[int]:
        if self.inter_measurement_period is None:
            return None
        return int(round(self.inter_measurement_period * 1000))

    @property
    def max_frequency(self) -> float:
        """Highest rate at which the chip can produce fresh measurements."""
        period: float = self.inter_measurement_period or self.timing_budget
        return 1.0 / max(period, self.timing_budget)

    def validate(self, sensor_model: str):
        """Raise if these knobs cannot be programmed onto the chip. Called before the
        sensor is probed, so a bad configuration does not look like missing hardware.
        """
        if sensor_model != "VL53L1X":
            return
        valid = self.valid_timing_budgets_ms(self.mode)
        if self.timing_budget_ms not in valid:
            raise ValueError(
                f"Timing budget of {self.timing_budget_ms}ms is not supported in mode "
                f"{ToFAccuracyModeVL53L1X(self.mode).name}. "
                f"Supported budgets (ms): {list(valid)}."
            )
        period_ms = self.inter_measurement_period_ms
        if period_ms is not None and period_ms < self.timing_budget_ms:
            raise ValueError(
                f"Inter-measurement period of {period_ms}ms is shorter than the timing "
                f"budget of {self.timing_budget_ms}ms. The chip cannot start a "
                f"measurement before the previous one is finished."
            )

    @staticmethod
    def valid_timing_budgets_ms(mode: int) -> Tuple[int, ...]:
        try:
            return VL53L1X_TIMING_BUDGETS_MS[ToFAccuracyModeVL53L1X(mode)]
        except ValueError:
            return ()

    @staticmethod
    def from_string(mode: str, sensor_model: str = "VL53L0X"):
        ms = 1 / 1000

        profiles = {
            "VL53L0X": {
                "GOOD": ToFAccuracy(ToFAccuracyModeVL53L0X.GOOD, 33 * ms, 1.2),
                "BETTER": ToFAccuracy(ToFAccuracyModeVL53L0X.BETTER, 66 * ms, 1.2),
                "BEST": ToFAccuracy(ToFAccuracyModeVL53L0X.BEST, 200 * ms, 1.2),
                "LONG_RANGE": ToFAccuracy(ToFAccuracyModeVL53L0X.LONG_RANGE, 33 * ms, 2.0),
                "HIGH_SPEED": ToFAccuracy(ToFAccuracyModeVL53L0X.HIGH_SPEED, 20 * ms, 1.2),
            },
            "VL53L1X": {
                # 1.36 and 3.6 are the Adafruit library's figures. ST quotes 1.3m and 4m,
                # and the 4m needs a 140ms budget in the dark against a white target
                "SHORT_RANGE": ToFAccuracy(
                    mode=ToFAccuracyModeVL53L1X.SHORT_RANGE,
                    timing_budget=20 * ms,
                    max_range=1.36,
                    min_range=0.04,
                    fov=np.deg2rad(27),
                    inter_measurement_period=20 * ms,
                ),
                "LONG_RANGE": ToFAccuracy(
                    mode=ToFAccuracyModeVL53L1X.LONG_RANGE,
                    timing_budget=33 * ms,
                    max_range=3.6,
                    min_range=0.04,
                    fov=np.deg2rad(27),
                    inter_measurement_period=33 * ms,
                ),
            },
        }
        if sensor_model not in profiles:
            raise ValueError(
                f"Unknown sensor model '{sensor_model}'. Known models: {sorted(profiles)}."
            )
        if mode not in profiles[sensor_model]:
            raise ValueError(
                f"Unknown mode '{mode}' for sensor model '{sensor_model}'. "
                f"Known modes: {sorted(profiles[sensor_model])}."
            )
        return profiles[sensor_model][mode]


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

    @property
    def data_ready(self) -> bool:
        """Whether the sensor holds a measurement that has not been read yet. Drivers
        that cannot tell report True, so callers fall back to plain polling.
        """
        return True

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
