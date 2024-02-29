#!/usr/bin/env python3

import asyncio
import dataclasses
import time
from enum import IntEnum
from typing import Optional

import argparse

from button_driver import ButtonEvent, ButtonDriver
from dt_device_utils.device import shutdown_device

# from display_renderer import (
#     PAGE_SHUTDOWN,
#     MonoImageFragmentRenderer,
#     REGION_BODY,
#     DisplayROI,
# )
# from display_renderer.text import monospace_screen

from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.node import Node
from dtps import DTPSContext
from dtps_http import RawData
from duckietown_messages.sensors.button_event import InteractionEvent


@dataclasses.dataclass
class ButtonDriverNodeConfiguration(NodeConfiguration):
    led_gpio_pin: int
    signal_gpio_pin: int


class ButtonDriverNode(Node):
    """
    This class implements the communication logic with a wheel encoder sensor via GPIO pins.
    """

    _TIME_HOLD_3S = 3
    _TIME_HOLD_10S = 10

    def __init__(self, config: str):
        super().__init__(
            name="power-button",
            kind=NodeType.DRIVER,
            description="Power button driver",
        )
        # load configuration
        self.configuration: ButtonDriverNodeConfiguration = (ButtonDriverNodeConfiguration.
                                                             from_name(self.package, config))
        # queues
        self._queue: Optional[DTPSContext] = None
        # create a ButtonDriver sensor handler
        self._sensor: ButtonDriver = ButtonDriver(
            self.configuration.led_gpio_pin,
            self.configuration.signal_gpio_pin,
            self._event_cb
        )
        # turn ON the LED
        self._sensor.led.on()
        # create event holder
        self._ongoing_event = None
        # create screen renderer
        # self._renderer = BatteryShutdownConfirmationRenderer()

    def _event_cb(self, event: ButtonEvent):
        # wait for DTPS to initialize
        if self._queue is None:
            return
        # create partial event
        if event == ButtonEvent.PRESS:
            # create new partial event
            self._ongoing_event = time.time()
            return
        # this is a RELEASE event
        if self._ongoing_event is None:
            # we missed it, well, next time!
            return
        # create new full event
        duration = time.time() - self._ongoing_event
        # clear ongoing event
        self._ongoing_event = None
        # analyze event
        # - single click
        if duration < 0.5:
            self.call_soon(self._react, InteractionEvent.SINGLE_CLICK)
        # - held for 3 secs
        elif self._TIME_HOLD_3S < duration < 2 * self._TIME_HOLD_3S:
            self.call_soon(self._react, InteractionEvent.HELD_3SEC)
        # - held for 10 secs
        elif self._TIME_HOLD_10S < duration:
            self.call_soon(self._react, InteractionEvent.HELD_10SEC)

    async def _react(self, event: InteractionEvent):
        # publish
        data: dict = {
            "data": event.value,
        }
        rdata: RawData = RawData.cbor_from_native_object(data)
        await self._queue.publish(rdata)
        # return control to the event loop
        await asyncio.sleep(0.01)
        # react
        if event in [InteractionEvent.HELD_3SEC, InteractionEvent.HELD_10SEC]:
            # blink top power button as a confirmation, too
            self._sensor.led.blink(duration=3, frequency=2)

            # turn off front and back LEDs
            # TODO: implement this once we have the patient contexts in DTPS

            # init shutdown sequence
            res = shutdown_device()
            if not res:
                self.logerr("Could not initiate the shutdown sequence")

    async def worker(self):
        await self.dtps_init(self.configuration)
        # create sensor queue
        self._queue = await (self.context / "out" / "event").queue_create()
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "sensors" / "power-button").expose(self._queue)
        # publish no event
        await self._queue.publish(RawData.cbor_from_native_object({
            "data": InteractionEvent.NOTHING.value,
        }))
        # sit and wait for the callbacks to come in
        while not self.is_shutdown:
            # wait
            await asyncio.sleep(1.0)

    def on_shutdown(self):
        if hasattr(self, "_button"):
            # noinspection PyBroadException
            try:
                self._button.shutdown()
            except BaseException:
                pass


# class BatteryShutdownConfirmationRenderer(MonoImageFragmentRenderer):
#     def __init__(self):
#         super(BatteryShutdownConfirmationRenderer, self).__init__(
#             name=f"__battery_shutdown_confirmation__",
#             page=PAGE_SHUTDOWN,
#             region=REGION_BODY,
#             roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
#             ttl=-1,  # on shutdown, just need one fixed screen
#         )
#
#         contents = monospace_screen((self.roi.h, self.roi.w), "Shutting down", scale="hfill", align="center")
#         self.data[:, :] = contents


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True, help="Name of the configuration")
    args: argparse.Namespace = parser.parse_args()
    # create node
    node: ButtonDriverNode = ButtonDriverNode(config=args.config)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
