from tof_driver.include.tof_driver.tof_driver_abs import ToFDriverAbs, ToFAccuracy

from dt_vl53l0x import VL53L0X


class ToFDriver(ToFDriverAbs):

    def __init__(self, name: str, accuracy: ToFAccuracy, i2c_bus: int, i2c_address: int):
        super(ToFDriver, self).__init__(name, accuracy)
        self._sensor = VL53L0X(i2c_bus=i2c_bus, i2c_address=i2c_address)

    def start(self):
        self._sensor.open()
        self._sensor.start_ranging(self._accuracy.mode)

    def get_distance(self) -> float:
        return self._sensor.get_distance()

    def shutdown(self):
        self._sensor.stop_ranging()
