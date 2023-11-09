"""
The wheel encoder package contains a single class (i.e., WheelEncoderDriver) that is responsible
for talking to a single wheel encoder.

.. autoclass:: tof_driver.WheelEncoderDriver

"""

from dt_robot_utils import get_robot_hardware, RobotHardware

from .tof_driver_abs import ToFAccuracy, ToFDriverAbs

if get_robot_hardware() != RobotHardware.VIRTUAL:
    # this breaks if imported when running on a virtual robot
    from .tof_driver import ToFDriver
else:
    from .virtual_tof_driver import VirtualToFDriver as ToFDriver

