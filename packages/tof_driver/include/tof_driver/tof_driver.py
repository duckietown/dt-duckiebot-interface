from typing import Optional

from tof_driver.tof_driver_abs import ToFDriverAbs, ToFAccuracy

from adafruit_extended_bus import ExtendedI2C
from adafruit_vl53l0x import VL53L0X
from adafruit_vl53l1x import VL53L1X


class ToFDriverVL53L0X(ToFDriverAbs):

    def __init__(self, name: str, accuracy: ToFAccuracy, i2c_bus: int, i2c_address: int):
        super(ToFDriverVL53L0X, self).__init__(name, accuracy)
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
        

class ToFDriverVL53L1X(ToFDriverAbs):

    def __init__(self, name: str, accuracy: ToFAccuracy, i2c_bus: int, i2c_address: int):
        super(ToFDriverVL53L1X, self).__init__(name, accuracy)
        self._i2c_bus: int = i2c_bus
        self._i2c_address: int = i2c_address
        self._sensor: Optional[VL53L1X] = None

    def setup(self):
        bus: ExtendedI2C = ExtendedI2C(self._i2c_bus)
        addresses = bus.scan()
        decimal_addresses = ', '.join(str(addr) for addr in addresses)
        hex_addresses = ', '.join(hex(addr) for addr in addresses)
        print(f"Devices on bus {self._i2c_bus}: Decimal - {decimal_addresses}, Hexadecimal - {hex_addresses}")
        print(f"{self.__class__.__name__}: Setting up sensor on bus {self._i2c_bus} at address {self._i2c_address}")
        self._sensor = VL53L1X(bus, address=self._i2c_address)
        # set accuracy mode
        self._sensor.distance_mode = self._accuracy.mode

    def start(self):
        self._sensor.start_ranging()

    def get_distance(self) -> float:
        # The sensor returns the distance in centimeters, we convert it to millimeters
        distance_cm = self._sensor.distance
        
        if distance_cm is not None:
            return max(0, distance_cm*10)
        else:
            # If the sensor returns
            print(f"{self.__class__.__name__}: Sensor returned None, returning infinity.")
            return float('inf')
        
    def stop(self):
        self._sensor.stop_ranging()

    def release(self):
        self._sensor = None
