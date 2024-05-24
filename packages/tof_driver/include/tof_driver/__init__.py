"""
The time of flight package contains a single class (i.e., To) that is responsible
for talking to a single wheel encoder.

.. autoclass:: tof_driver.WheelEncoderDriver

"""

from dt_robot_utils import get_robot_hardware, RobotHardware, get_robot_type, RobotType, RobotConfiguration
from dt_robot_utils.robot import get_robot_configuration

from .tof_driver_abs import ToFAccuracy, ToFDriverAbs

if get_robot_hardware() != RobotHardware.VIRTUAL:
    # this breaks if imported when running on a virtual robot
    if get_robot_configuration() == RobotConfiguration.DD24:
        from .tof_driver import ToFDriverVL53L1X as ToFDriver
    else:
        from .tof_driver import ToFDriverVL53L0X as ToFDriver
else:
    if get_robot_type() == RobotType.DUCKIEDRONE:
        from .gazebo_tof_driver import GazeboToFDriver as ToFDriver
    else:
        from .virtual_tof_driver import VirtualToFDriver as ToFDriver
