from dt_robot_rest_api.constants import HTTP_PORTS
from dt_robot_utils import RobotType

from dt_robot_rest_api.actions.estop import estop_bp


blueprints = [
    estop_bp
]
HTTP_PORT = HTTP_PORTS[RobotType.DUCKIEBOT]
