import dataclasses
from logging import Logger
from typing import List, Optional

import board
import adafruit_mpu6050
from adafruit_mpu6050 import MPU6050
from smbus2 import SMBus  # NEW

from dtps import DTPSContext
from .exceptions import DeviceNotFound

from .types import I2CConnector


class CalibratedMPU6050:
    WHO_AM_I_REG = 0x75
    ALLOWED_IDS = (0x68, 0x98)        # MPU-6050, ICM-2068x

    def __init__(self, i2c_connectors: List[I2CConnector], cxt: DTPSContext, logger: Logger):
        self._i2c_connectors: List[I2CConnector] = i2c_connectors
        self._cxt: DTPSContext = cxt
        self._logger: Logger = logger
        # calibration
        # TODO: implement loading from file and calibration service
        self._gyroscope_offsets: List[float] = [0.0, 0.0, 0.0]
        self._accelerometer_offsets: List[float] = [0.0, 0.0, 0.0]
        self._thermometer_offset: float = 0.0
        # imu sensor
        self.sensor: Optional[MPU6050] = self._find_sensor()

    @property
    def linear_accelerations(self) -> List[float]:
        # apply offsets
        return [
            v - self._accelerometer_offsets[i]
            for i, v in enumerate(self.sensor.acceleration)
        ]

    @property
    def angular_velocities(self) -> List[float]:
        # apply offsets
        return [
            v - self._gyroscope_offsets[i]
            for i, v in enumerate(self.sensor.gyro)
        ]

    @property
    def temperature(self) -> float:
        return self.sensor.temperature - self._thermometer_offset

    def calibrate_offsets(self):
        self._gyroscope_offsets = list(self.sensor.gyro)
        self._accelerometer_offsets = list(self.sensor.acceleration)
        self._thermometer_offset = self.sensor.temperature
        self._logger.info("IMU zeroed with\n"
                          "\tACC: X:{:.2f}, Y: {:.2f}, Z: {:.2f} m/s^2\t|"
                          "\tGyro X:{:.2f}, Y: {:.2f}, Z: {:.2f} degrees/s"
                          "".format(*self._accelerometer_offsets, *self._gyroscope_offsets))

    def _find_sensor(self) -> MPU6050:
        """
        Scan each (bus, address) pair in ~connectors.
        Accept devices whose WHO_AM_I register is 0x68 (MPU-6050)
        or 0x98 (ICM-2068x clone).  Anything else is ignored.
        """
        for connector in self._i2c_connectors:
            bus_n = connector.bus
            addr = connector.address
            conn_str = "[bus:{bus}](0x{address:02X})".format(**dataclasses.asdict(connector))
            self._logger.info(f"Trying to open device on connector {conn_str}")

            # --- read WHO_AM_I first -----------------------------------
            try:
                with SMBus(bus_n) as bus:
                    who_am_i = bus.read_byte_data(addr, self.WHO_AM_I_REG)
            except FileNotFoundError:
                self._logger.warning(f"I²C bus {bus_n} does not exist")
                continue
            except OSError:
                self._logger.warning(f"No devices found on connector {conn_str}, but the bus exists")
                continue

            if who_am_i not in self.ALLOWED_IDS:
                self._logger.warning(
                    f"Unknown IMU WHO_AM_I=0x{who_am_i:02X} on connector {conn_str}, skipping"
                )
                continue

            # --- patch driver constant and instantiate -----------------
            adafruit_mpu6050._MPU6050_DEVICE_ID = who_am_i
            try:
                sensor = MPU6050(board.I2C(), address=addr)
            except Exception as e:
                self._logger.warning(f"Driver rejected device on {conn_str}: {e}")
                continue

            self._logger.info(
                f"Device (WHO_AM_I=0x{who_am_i:02X}) found on connector {conn_str}"
            )
            return sensor

        # none of the connectors yielded a valid sensor
        raise DeviceNotFound()