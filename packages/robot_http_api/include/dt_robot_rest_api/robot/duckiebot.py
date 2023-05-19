from dt_robot_rest_api.constants import HTTP_PORTS
from dt_robot_utils import RobotType

from dt_robot_rest_api.actions.estop import estop_bp
from dt_robot_rest_api.actions.car import car_bp
from dt_robot_rest_api.actions.logs import logs_bp


blueprints = [estop_bp, car_bp, logs_bp]
HTTP_PORT = HTTP_PORTS[RobotType.DUCKIEBOT]
