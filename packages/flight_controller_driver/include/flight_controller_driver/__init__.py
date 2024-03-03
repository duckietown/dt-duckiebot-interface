"""
The flight controller package contains a single class (i.e., FlightController) that is responsible
for talking to a betaflight-based flight controller.

.. autoclass:: flight_controller_driver.FlightController

"""

from dt_robot_utils import get_robot_hardware, RobotHardware

if get_robot_hardware() == RobotHardware.VIRTUAL:
    # this breaks if imported when running on a virtual robot
    from .flight_controller_virtual import FlightControllerSITL as FlightController
else:
    from .flight_controller_physical import FlightController

