import dataclasses
from logging import Logger
from typing import List, Optional

import board
import adafruit_mpu6050
from adafruit_mpu6050 import MPU6050

from dtps import DTPSContext
from .exceptions import DeviceNotFound

from .types import I2CConnector


class CalibratedMPU6050:

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

    def _find_sensor(self) -> Optional[MPU6050]:
        for connector in self._i2c_connectors:
            conn: str = "[bus:{bus}](0x{address:02X})".format(**dataclasses.asdict(connector))
            self._logger.info(f"Trying to open device on connector {conn}")
            # overwrite Adafruit default device ID
            adafruit_mpu6050._MPU6050_DEVICE_ID = connector.address
            # noinspection PyBroadException
            try:
                sensor = MPU6050(board.I2C())
            except Exception:
                self._logger.warning(f"No devices found on connector {conn}, but the bus exists")
                continue
            self._logger.info(f"Device found on connector {conn}")
            return sensor
        # if no sensor found, raise
        raise DeviceNotFound()
