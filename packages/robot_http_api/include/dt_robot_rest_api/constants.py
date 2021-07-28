from dt_robot_utils import RobotType

HTTP_PORTS = {
    RobotType.UNKNOWN: None,
    RobotType.DUCKIEBOT: 8090,
    RobotType.WATCHTOWER: 8091,
    RobotType.TRAFFIC_LIGHT: 8092,
    RobotType.DUCKIETOWN: 8093,
    RobotType.DUCKIEDRONE: 8094
}
