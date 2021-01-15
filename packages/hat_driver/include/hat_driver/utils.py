from dt_robot_utils import RobotConfiguration, get_robot_configuration

from .hat import HATv1, HATv2, HATv3


_ROBOT_CONFIGURATION_TO_HAT = {
    RobotConfiguration.DB18: HATv1,
    RobotConfiguration.DB19: HATv2,
    RobotConfiguration.DB20: HATv2,
    RobotConfiguration.DB21M: HATv3,
}
_DEFAULT_HAT = HATv1


def from_env():
    rcfg = get_robot_configuration()
    DTHAT = _ROBOT_CONFIGURATION_TO_HAT.get(rcfg, _DEFAULT_HAT)
    return DTHAT
