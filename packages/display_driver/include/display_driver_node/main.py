#!/usr/bin/env python3

import dataclasses
import time
from typing import Optional, List

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
from duckietown_messages.geometry_2d.roi import ROI
from duckietown_messages.utils.exceptions import DataDecodingError

from display_hardware_test import DisplayHardwareTest


@dataclasses.dataclass
class DisplayNodeConfiguration(NodeConfiguration):
    bus: Optional[int] = None
    buses: Optional[List[int]] = None
    address: int = 0x3C
    frequency: float = 1.0


BOOTING_SCREEN: DisplayFragment = DisplayFragment(
    name="__booting__",
    region=DisplayRegionID.BODY,
    page=PAGE_INIT,
    content=Image.from_np(
        monospace_screen((32, 128), " Loading... ", scale="hfill"),
        encoding="mono8"
    ),
    location=ROI(x=0, y=8, width=128, height=32),
    z=0,
    ttl=-1
)

SHUTTING_DOWN_SCREEN: DisplayFragment = DisplayFragment(
    name="__shutting_down__",
    region=DisplayRegionID.BODY,
    page=PAGE_SHUTDOWN,
    content=Image.from_np(
        monospace_screen((32, 128), " Shutting down... ", scale="hfill"),
        encoding="mono8"
    ),
    location=ROI(x=0, y=8, width=128, height=32),
    z=0,
    ttl=-1
)


class DisplayNode(Node):

    def __init__(self, config: str, actuator_name: str):
        node_name: str = f"display_driver_{actuator_name}"
        super().__init__(
            name=node_name,
            kind=NodeType.DRIVER,
            description="LCD display driver",
        )
        self.actuator_name: str = actuator_name
        # configuration
        self.configuration: DisplayNodeConfiguration = DisplayNodeConfiguration.from_name(
            self.package, node_name, config)
        # resolve list of buses to try (support both 'bus' and 'buses' config keys)
        buses = self.configuration.buses if self.configuration.buses else [self.configuration.bus]
        # create display driver — try each bus in order
        self._display: Optional[SSD1306Display] = None
        for bus in buses:
            try:
                self._display = SSD1306Display(
                    bus,
                    self.configuration.address,
                    self.configuration.frequency,
                    self.logger
                )
                self.logger.info(f"Display initialized on I2C bus {bus}")
                break
            except Exception as e:
                self.logger.warning(f"Failed to initialize display on I2C bus {bus}: {e}")
        if self._display is None:
            raise RuntimeError(f"Failed to initialize display on any of the I2C buses: {buses}")
        # running test flag
        self.running_test: bool = False

    async def cb_fragments(self, data: RawData):
        """
        Callback processing incoming fragments.
        """
        if self.running_test:
            return
        try:
            fragments: DisplayFragments = DisplayFragments.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # update the display
        for fragment in fragments.fragments:
            self._display.add_fragment(fragment)

    async def cb_button_events(self, data: RawData):
        """
        Callback processing incoming button events.
        """
        if self.running_test:
            return
        try:
            event: ButtonEvent = ButtonEvent.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # ---
        if self._display.page != PAGE_SHUTDOWN:
            if event.type == InteractionEvent.SINGLE_CLICK:
                # switch to the next page
                self._display.next_page()
            elif event.type in [InteractionEvent.HELD_3SEC, InteractionEvent.HELD_10SEC]:
                # switch to shut down page
                self._display.page = PAGE_SHUTDOWN

    async def worker(self):
        await self.dtps_init(self.configuration)
        # create fragments queue
        self.fragments_queue = await (self.context / "in" / "fragments").queue_create()
        test_in_queue = await (self.context / "test" / "in").queue_create()
        test_out_queue = await (self.context / "test" / "out").queue_create()
        # test
        hardware_test = DisplayHardwareTest(self, test_out_queue, self._display)
        # subscribe to fragments
        await self.fragments_queue.subscribe(self.cb_fragments)
        await test_in_queue.subscribe(hardware_test.on_run_test)
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "actuator" / "display" / self.actuator_name / "fragments").expose(self.fragments_queue)
        await (self.switchboard / "actuator" / "display" / self.actuator_name / "test" / "in").expose(test_in_queue)
        await (self.switchboard / "actuator" / "display" / self.actuator_name / "test" / "out").expose(test_out_queue)
        # publish the initial state
        timestamp = time.time()
        header = Header(timestamp=timestamp)
        await self.fragments_queue.publish(DisplayFragments(
            header=header,
            fragments=(BOOTING_SCREEN, SHUTTING_DOWN_SCREEN)
        ).to_rawdata())
        # run forever
        await self.join()

    @sidecar
    async def register_button_events(self):
        await self.switchboard_ready.wait()
        # create button event queue
        button: DTPSContext = await (self.switchboard / "sensor" / "power_button" / self.actuator_name / "event").until_ready()
        # subscribe to button events
        await button.subscribe(self.cb_button_events)


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--actuator-name", type=str, required=True, help="Name of the actuator")
    parser.add_argument("--config", type=str, required=True, help="Name of the configuration")
    args: argparse.Namespace = parser.parse_args()
    # create node
    node: DisplayNode = DisplayNode(config=args.config, actuator_name=args.actuator_name)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
