"""

    rgb_led
    -------

    Describe `rgb_leb` here...

    .. autoclass:: rgb_led.RGB_LED

"""

from dt_robot_utils import get_robot_hardware, RobotHardware


if get_robot_hardware() != RobotHardware.VIRTUAL:
    # this breaks if imported when running on a virtual robot
    from .rgb_led import RGBLED

from .virtual_rgb_led import VirtualRGBLED
