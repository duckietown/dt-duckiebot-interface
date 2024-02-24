#!/usr/bin/env python3

import dataclasses
import time
from typing import Type, cast, List

import argparse

from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.node import Node
from dt_robot_utils import RobotHardware, get_robot_hardware
from dtps import DTPSContext
from dtps_http import RawData
from leds_driver.leds_driver_abs import LEDsDriverAbs

if get_robot_hardware() == RobotHardware.VIRTUAL:
    from leds_driver.virtual_leds_driver import VirtualLEDsDriver
    LEDsDriver: Type[LEDsDriverAbs] = VirtualLEDsDriver
else:
    from leds_driver.leds_driver import PWMLEDsDriver
    LEDsDriver: Type[LEDsDriverAbs] = PWMLEDsDriver


uint8 = int


@dataclasses.dataclass
class LEDsDriverNodeConfiguration(NodeConfiguration):
    initial_pattern: List[List[uint8]]


class LEDsDriverNode(Node):
    """
    Node handling the robot lights.

    Subscribes to a stream of light patterns and implements them.
    """

    def __init__(self, config: str):
        super(LEDsDriverNode, self).__init__(
            name="leds",
            kind=NodeType.DRIVER,
            description="Robot lights driver",
        )

        # load configuration
        self.configuration: LEDsDriverNodeConfiguration = (LEDsDriverNodeConfiguration.
                                                           from_name(self.package, config))
        # setup the driver
        self.driver: LEDsDriverAbs = LEDsDriver(debug=False)

    async def cb_pattern(self, data: RawData):
        """
        Callback that implements a given light pattern.
        """
        lights: list = cast(list, data.get_as_native_object())
        if not isinstance(lights, list):
            self.logwarn("Received an invalid lights pattern. Expected a list of 5 RGB arrays, one for each light. "
                         "Received '{lights}' instead.")
            return
        for i, color in enumerate(lights):
            if not isinstance(color, list) or len(color) not in [3, 4]:
                self.logwarn(f"Received an invalid RGB value for light ${i}. "
                             f"Expected an RGB array (i.e., a list of 3 or 4 uint8 values). "
                             f"Received '{color}' instead.")
                return
            # apply alpha
            if len(color) == 4:
                alpha: float = color[3] / 255.
                color = [int(v * alpha) for v in color[:3]]
            # realize RGB
            self.driver.set_rgb(i, color)

    async def worker(self):
        await self.dtps_init(self.configuration)
        # create RGB queue IN
        rgb_in: DTPSContext = await (self.context / "in" / "rgb").queue_create()
        # subscribe to RGB patterns
        await rgb_in.subscribe(self.cb_pattern)
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "actuator" / "leds" / "rgb").expose(rgb_in)
        # apply initial state
        await rgb_in.publish(RawData.cbor_from_native_object(self.configuration.initial_pattern))
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
