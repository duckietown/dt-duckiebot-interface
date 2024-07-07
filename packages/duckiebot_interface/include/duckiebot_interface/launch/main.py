import sys
from typing import List

import argparse
from launch import LaunchDescription, LaunchService

from dt_robot_utils import get_robot_name, get_robot_hardware, get_robot_type, get_robot_configuration
from launch_duckietown.actions import Node


def _tof_node(sensor_name: str, robot_type: str, config: str) -> Node:
    config: str = f"{robot_type}/{sensor_name}/{config}.yaml"
    return Node(
        package="tof_driver",
        module="tof_driver_node.main",
        config=config,
        name=f"tof-{sensor_name}",
        arguments=["--sensor-name", sensor_name]
    )


def generate_launch_description(args: argparse.Namespace) -> LaunchDescription:
    # robot info
    robot_name: str = get_robot_name()
    robot_hardware: str = get_robot_hardware().str
    robot_type: str = get_robot_type().str
    robot_configuration: str = get_robot_configuration().str
    # create nodes
    nodes: List[Node] = []
    # camera node
    if args.camera:
        config: str = f"{robot_type}/{robot_hardware}/{args.camera_config}.yaml"
        nodes.append(Node(
            package="camera_driver",
            module=f"camera_driver_node.{robot_hardware}",
            name="camera_driver",
            config=config
        ))
    # tof node
    if args.tof:
        if robot_type == "duckiebot" and robot_configuration in ["DB21M", "DB21J", "DBR", "DBR5"]:
            nodes.append(_tof_node("front_center", robot_type, args.tof_config))
    # wheels node
    if args.wheels:
        config: str = f"{args.wheels_config}.yaml"
        nodes.append(Node(
            package="wheels_driver",
            module="wheels_driver_node.main",
            name="wheels_driver",
            config=config
        ))
    # display node
    if args.display:
        config: str = f"{args.display_config}.yaml"
        nodes.append(Node(
            package="display_driver",
            module="display_driver_node.main",
            name="display_driver",
            config=config
        ))
    # wheel encoder nodes
    if args.wheel_encoders:
        if robot_configuration in ["DB21M", "DB21J", "DBR"]:
            for side in ["left", "right"]:
                config: str = f"{side}/{args.wheel_encoders_config}.yaml"
                nodes.append(Node(
                    package="wheel_encoder_driver",
                    module=f"wheel_encoder_driver_node.main",
                    name=f"wheel_encoder_{side}",
                    config=config,
                    arguments=["--side", side]
                ))
    # leds node
    if args.leds:
        config: str = f"{args.leds_config}.yaml"
        nodes.append(Node(
            package="leds_driver",
            module="leds_driver_node.main",
            name="leds_driver",
            config=config
        ))
    # button node
    if args.button:
        config: str = f"{args.button_config}.yaml"
        nodes.append(Node(
            package="button_driver",
            module="button_driver_node.main",
            name="button_driver",
            config=config
        ))
    # imu node
    if args.imu:
        config: str = f"{robot_type}/{args.imu_config}.yaml"
        nodes.append(Node(
            package="imu_driver",
            module="imu_driver_node.main",
            name="imu_driver",
            config=config
        ))
    # display renderer node
    if args.display_renderer:
        config: str = f"{args.display_renderer_config}.yaml"
        nodes.append(Node(
            package="display_renderer",
            module="display_renderer_node.main",
            name="display_renderer",
            config=config
        ))
    # ---
    return LaunchDescription(nodes)


if __name__ == '__main__':
    # arguments
    parser = argparse.ArgumentParser()
    # camera
    parser.add_argument("--camera", action="store_true", default=False, help="Run camera driver node")
    parser.add_argument("--camera-config", type=str, default="default", help="Camera configuration file to use")
    # tof
    parser.add_argument("--tof", "--tofs", action="store_true", default=False, help="Run time-of-flight sensor node(s)")
    parser.add_argument("--tof-config", type=str, default="default", help="Time-of-flight configuration file to use")
    # wheels
    parser.add_argument("--wheels", action="store_true", default=False, help="Run wheels driver node")
    parser.add_argument("--wheels-config", type=str, default="default", help="Wheels configuration file to use")
    # wheel encoders
    parser.add_argument("--wheel-encoders", action="store_true", default=False, help="Run wheel encoder driver nodes")
    parser.add_argument("--wheel-encoders-config", type=str, default="default",
                        help="Wheel encoders configuration file to use")
    # leds
    parser.add_argument("--leds", action="store_true", default=False, help="Run LEDs driver node")
    parser.add_argument("--leds-config", type=str, default="default", help="LEDs configuration file to use")
    # power button
    parser.add_argument("--button", action="store_true", default=False, help="Run power button driver node")
    parser.add_argument("--button-config", type=str, default="default", help="Power button configuration file to use")
    # display
    parser.add_argument("--display", action="store_true", default=False, help="Run LCD display driver node")
    parser.add_argument("--display-config", type=str, default="default", help="LCD display configuration file to use")
    # imu
    parser.add_argument("--imu", action="store_true", default=False, help="Run IMU driver node")
    parser.add_argument("--imu-config", type=str, default="default", help="IMU configuration file to use")
    # display renderer
    parser.add_argument("--display-renderer", action="store_true", default=False, help="Run display renderer node")
    parser.add_argument("--display-renderer-config", type=str, default="default",
                        help="Renderer configuration file to use")
    # ---
    parsed: argparse.Namespace = parser.parse_args()

    # ls = LaunchService(argv=sys.argv, debug=True)  # Use this instead to get more debug messages.
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description(parsed))
    sys.exit(ls.run())
