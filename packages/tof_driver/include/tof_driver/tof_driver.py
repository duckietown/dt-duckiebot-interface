from typing import Optional

from tof_driver.tof_driver_abs import ToFDriverAbs, ToFAccuracy

from adafruit_extended_bus import ExtendedI2C
from .adafruit_vl53l0x import VL53L0X
from adafruit_vl53l1x import VL53L1X


class ToFDriverVL53L0X(ToFDriverAbs):

    def __init__(self, name: str, accuracy: ToFAccuracy, i2c_bus: int, i2c_address: int):
        super(ToFDriverVL53L0X, self).__init__(name, accuracy)
        self._i2c_bus: int = i2c_bus
        self._i2c_address: int = i2c_address
        self._sensor: Optional[VL53L0X] = None

    def setup(self):
        bus: ExtendedI2C = ExtendedI2C(self._i2c_bus)
        self._sensor = VL53L0X(bus, address=self._i2c_address, strict_check=False)
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

    # registers the Adafruit library does not expose
    _INTER_MEASUREMENT_PERIOD = 0x006C
    _OSC_CALIBRATE_VAL = 0x00DE

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
        self._accuracy.validate("VL53L1X")
        self._sensor = VL53L1X(bus, address=self._i2c_address)
        # mode first: the library re-applies the timing budget whenever the mode changes
        self._sensor.distance_mode = self._accuracy.mode
        self._sensor.timing_budget = self._accuracy.timing_budget_ms
        self._set_inter_measurement_period()

    def _set_inter_measurement_period(self):
        """Program how often the chip starts a new measurement. The library leaves this
        register at 100ms, capping the sensor at 10Hz whatever the budget is, and offers
        no public API for it, so it is written directly the way ST's own driver does.
        """
        period_ms = self._accuracy.inter_measurement_period_ms
        if period_ms is None:
            return
        osc = int.from_bytes(self._sensor._read_register(self._OSC_CALIBRATE_VAL, 2), "big") & 0x3FF
        if osc == 0:
            print(f"{self.__class__.__name__}: oscillator calibration value is 0, "
                  f"leaving the inter-measurement period untouched.")
            return
        raw: int = int(osc * period_ms * 1.075)
        self._sensor._write_register(self._INTER_MEASUREMENT_PERIOD, raw.to_bytes(4, "big"))

    def start(self):
        self._sensor.start_ranging()

    @property
    def data_ready(self) -> bool:
        return self._sensor.data_ready

    def get_distance(self) -> float:
        # The sensor returns the distance in centimeters, we convert it to millimeters
        distance_cm = self._sensor.distance
        # acknowledge it, or data_ready stays high and every read looks fresh
        self._sensor.clear_interrupt()

        if distance_cm is not None:
            return max(0, distance_cm*10)
        else:
            # no target in range, or the return was rejected as too noisy
            return float('inf')

    def stop(self):
        self._sensor.stop_ranging()

    def release(self):
        self._sensor = None
