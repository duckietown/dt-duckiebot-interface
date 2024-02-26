"""
The wheel encoder package contains a single class (i.e., WheelEncoderDriver) that is responsible
for talking to a single wheel encoder.

.. autoclass:: wheel_encoder.WheelEncoderDriver

"""

from dt_robot_utils import get_robot_hardware, RobotHardware

from .wheel_encoder_abs import WheelDirection

if get_robot_hardware() != RobotHardware.VIRTUAL:
    # this breaks if imported when running on a virtual robot
    from .dagu_wheel_encoder_driver import DaguWheelEncoderDriver as WheelEncoderDriver
else:
    # this breaks if imported when running on a real robot
    from .virtual_wheel_encoder_driver import VirtualWheelEncoderDriver as WheelEncoderDriver
