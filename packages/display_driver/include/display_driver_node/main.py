#!/usr/bin/env python3

import dataclasses
from typing import List

import argparse

from display_driver.luma.ssd1306 import SSD1306Display
from display_driver.types.page import PAGE_SHUTDOWN, PAGE_INIT
from display_driver.types.regions import DisplayRegionID
from display_renderer import monospace_screen
from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.decorators import sidecar
from dt_node_utils.node import Node
from dtps import DTPSContext
from dtps_http import RawData
from duckietown_messages.actuators.display_fragment import DisplayFragment
from duckietown_messages.actuators.display_fragments import DisplayFragments
from duckietown_messages.sensors.button_event import ButtonEvent, InteractionEvent
from duckietown_messages.sensors.image import Image
from duckietown_messages.standard.header import Header
from duckietown_messages.standard.roi import ROI


@dataclasses.dataclass
class DisplayNodeConfiguration(NodeConfiguration):
    bus: int
    address: int
    frequency: float


BOOTING_SCREEN: List[DisplayFragment] = [
    DisplayFragment(
        header=Header(),
        name="__booting__",
        region=DisplayRegionID.BODY,
        page=PAGE_INIT,
        content=Image.from_np(
            monospace_screen((32, 128), " Loading... ", scale="hfill"),
            encoding="mono8",
            header=Header()
        ),
        location=ROI(header=Header(), x=0, y=8, width=128, height=32),
        z=0,
        ttl=-1,
    ),
]


class DisplayNode(Node):

    def __init__(self, config: str):
        super().__init__(
            name="display-driver",
            kind=NodeType.DRIVER,
            description="LCD display driver",
        )
        # configuration
        self._configuration: DisplayNodeConfiguration = DisplayNodeConfiguration.from_name(self.package, config)
        # create display driver
        self._display = SSD1306Display(
            self._configuration.bus,
            self._configuration.address,
            self._configuration.frequency,
            self.logger
        )

    async def cb_fragments(self, data: RawData):
        """
        Callback processing incoming fragments.
        """
        fragments: DisplayFragments = DisplayFragments.from_rawdata(data)
        # update the display
        for fragment in fragments.fragments:
            self._display.add_fragment(fragment)

    async def cb_button_events(self, data: RawData):
        """
        Callback processing incoming button events.
        """
        event: ButtonEvent = ButtonEvent.from_rawdata(data)
        if event.type == InteractionEvent.SINGLE_CLICK:
            # switch to the next page
            self._display.next_page()

        if event.type in [InteractionEvent.HELD_3SEC, InteractionEvent.HELD_10SEC]:
            # switch to shut down page
            self._display.page = PAGE_SHUTDOWN

    async def worker(self):
        await self.dtps_init(self._configuration)
        # create fragments queue
        fragments: DTPSContext = await (self.context / "in" / "fragments").queue_create()
        # subscribe to fragments
        await fragments.subscribe(self.cb_fragments)
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "actuator" / "display" / "fragments").expose(fragments)
        # publish the initial state
        await fragments.publish(DisplayFragments(
            header=Header(),
            fragments=BOOTING_SCREEN
        ).to_rawdata())
        # run forever
        await self.join()

    @sidecar
    async def register_button_events(self):
        await self.switchboard_ready.wait()
        # create button event queue
        button: DTPSContext = await (self.switchboard / "sensors" / "power-button").until_ready()
        # subscribe to button events
        await button.subscribe(self.cb_button_events)


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True, help="Name of the configuration")
    args: argparse.Namespace = parser.parse_args()
    # create node
    node: DisplayNode = DisplayNode(config=args.config)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
