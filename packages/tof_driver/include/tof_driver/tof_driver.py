from typing import Optional

from tof_driver.tof_driver_abs import ToFDriverAbs, ToFAccuracy

from adafruit_extended_bus import ExtendedI2C
from adafruit_vl53l0x import VL53L0X


class ToFDriver(ToFDriverAbs):

    def __init__(self, name: str, accuracy: ToFAccuracy, i2c_bus: int, i2c_address: int):
        super(ToFDriver, self).__init__(name, accuracy)
        self._i2c_bus: int = i2c_bus
        self._i2c_address: int = i2c_address
        self._sensor: Optional[VL53L0X] = None

    def setup(self):
        bus: ExtendedI2C = ExtendedI2C(self._i2c_bus)
        self._sensor = VL53L0X(bus, address=self._i2c_address)
        # set accuracy mode (in microseconds)
        self._sensor.measurement_timing_budget = int(self._accuracy.timing_budget * 10**6)

    def start(self):
        self._sensor.start_continuous()

    def get_distance(self) -> float:
        return max(0, self._sensor.range)

    def stop(self):
        self._sensor.stop_continuous()

    def release(self):
        self._sensor = None
