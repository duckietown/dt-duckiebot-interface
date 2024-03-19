#!/usr/bin/env python3

import dataclasses
import time
from typing import Type, cast, List, Dict

import argparse

from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.node import Node
from dt_robot_utils import RobotHardware, get_robot_hardware
from dtps import DTPSContext
from dtps_http import RawData
from duckietown_messages.actuators.car_lights import CarLights
from duckietown_messages.colors.rgba import RGBA
from duckietown_messages.utils.exceptions import DataDecodingError
from leds_driver.leds_driver_abs import LEDsDriverAbs

if get_robot_hardware() == RobotHardware.VIRTUAL:
    from leds_driver.virtual_leds_driver import VirtualLEDsDriver
    LEDsDriver: Type[LEDsDriverAbs] = VirtualLEDsDriver
else:
    from leds_driver.leds_driver import PWMLEDsDriver
    LEDsDriver: Type[LEDsDriverAbs] = PWMLEDsDriver


@dataclasses.dataclass
class LEDsDriverNodeConfiguration(NodeConfiguration):
    initial_pattern: Dict[str, List[float]]


class LEDsDriverNode(Node):
    """
    Node handling the robot lights.

    Subscribes to a stream of light patterns and implements them.
    """

    def __init__(self, config: str):
        node_name: str = "leds-driver"
        super(LEDsDriverNode, self).__init__(
            name=node_name,
            kind=NodeType.DRIVER,
            description="Robot lights driver",
        )

        # load configuration
        self.configuration: LEDsDriverNodeConfiguration = (LEDsDriverNodeConfiguration.
                                                           from_name(self.package, node_name, config))
        # setup the driver
        self.driver: LEDsDriverAbs = LEDsDriver(debug=False)

    async def cb_pattern(self, data: RawData):
        """
        Callback that implements a given light pattern.
        """
        try:
            msg: CarLights = CarLights.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # ---
        lights: List[RGBA] = [
            msg.front_left,
            RGBA.zero(),
            msg.front_right,
            msg.rear_right,
            msg.rear_left,
        ]
        for i, color in enumerate(lights):
            # apply alpha
            alpha: float = color[3]
            color: List[float] = [v * alpha for v in color[:3]]
            # realize RGB
            self.driver.set_rgb(i, color)

    async def worker(self):
        await self.dtps_init(self.configuration)
        # create RGB queue IN
        rgb_in: DTPSContext = await (self.context / "in" / "rgba").queue_create()
        # subscribe to RGB patterns
        await rgb_in.subscribe(self.cb_pattern)
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "actuator" / "leds" / "rgba").expose(rgb_in)
        # apply initial state
        msg: CarLights = CarLights(
            front_left=RGBA.from_list(self.configuration.initial_pattern["front_left"]),
            front_right=RGBA.from_list(self.configuration.initial_pattern["front_right"]),
            rear_left=RGBA.from_list(self.configuration.initial_pattern["rear_left"]),
            rear_right=RGBA.from_list(self.configuration.initial_pattern["rear_right"]),
        )
        await rgb_in.publish(msg.to_rawdata())
        # run forever
        await self.join()

    def on_shutdown(self):
        """
        Shutdown procedure.

        Publishes a zero velocity command at shutdown.
        """
        self.loginfo("Shutting down. Turning LEDs off.")
        self.driver.all_off()
        time.sleep(0.5)


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True, help="Name of the configuration")
    args: argparse.Namespace = parser.parse_args()
    # create node
    node: LEDsDriverNode = LEDsDriverNode(config=args.config)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
