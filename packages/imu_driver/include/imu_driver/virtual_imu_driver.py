from typing import Optional, List

from imu_driver.imu_driver_abs import IMUDriverAbs


class VirtualIMUDriver(IMUDriverAbs):

    def __init__(self, *_, **__):
        super(VirtualIMUDriver, self).__init__()

    @property
    def linear_accelerations(self) -> Optional[List[float]]:
        return None

    @property
    def angular_velocities(self) -> Optional[List[float]]:
        return None

    @property
    def temperature(self) -> Optional[float]:
        return None
