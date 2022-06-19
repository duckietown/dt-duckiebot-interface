from typing import Optional

from tof_driver.tof_driver_abs import ToFDriverAbs, ToFAccuracy

from dt_vl53l0x import VL53L0X


class ToFDriver(ToFDriverAbs):

    def __init__(self, name: str, accuracy: ToFAccuracy, i2c_bus: int, i2c_address: int):
        super(ToFDriver, self).__init__(name, accuracy)
        self._i2c_bus: int = i2c_bus
        self._i2c_address: int = i2c_address
        self._sensor: Optional[VL53L0X] = None

    def setup(self):
        self._sensor = VL53L0X(i2c_bus=self._i2c_bus, i2c_address=self._i2c_address)

    def start(self):
        self._sensor.open()
        self._sensor.start_ranging(self._accuracy.mode)

    def get_distance(self) -> float:
        return self._sensor.get_distance()

    def stop(self):
        self._sensor.stop_ranging()

    def release(self):
        self._sensor = None
