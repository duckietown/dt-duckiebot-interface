from dt_robot_utils import get_robot_type, RobotType

robot_type = get_robot_type()

if robot_type == RobotType.DUCKIEBOT:
    from .duckiebot import blueprints
    from .duckiebot import HTTP_PORT
elif robot_type == RobotType.WATCHTOWER:
    from .watchtower import blueprints
    from .watchtower import HTTP_PORT
elif robot_type == RobotType.TRAFFIC_LIGHT:
    from .traffic_light import blueprints
    from .traffic_light import HTTP_PORT
elif robot_type == RobotType.DUCKIETOWN:
    from .duckietown import blueprints
    from .duckietown import HTTP_PORT
elif robot_type == RobotType.DUCKIEDRONE:
    from .duckiedrone import blueprints
    from .duckiedrone import HTTP_PORT
else:
    blueprints = []
    HTTP_PORT = None


__all__ = [
    'blueprints',
    'HTTP_PORT'
]