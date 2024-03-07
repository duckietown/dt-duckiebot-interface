#!/usr/bin/env python3

import asyncio
import dataclasses
import os
import time
from typing import Optional, List

import argparse
from dtps import DTPSContext, PublisherInterface

from display_driver.types.page import PAGE_TOF
from display_driver.types.regions import REGION_BODY
from display_driver.types.roi import DisplayROI
from display_renderer import MonoImageFragmentRenderer, monospace_screen
from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.decorators import sidecar
from dt_node_utils.node import Node
from dt_robot_utils import get_robot_hardware, RobotHardware
from duckietown_messages.actuators.display_fragments import DisplayFragments
from duckietown_messages.sensors.range import Range
from duckietown_messages.sensors.range_finder import RangeFinder
from duckietown_messages.standard.header import Header
from tof_driver import ToFDriver, ToFAccuracy


@dataclasses.dataclass
class ToFNodeConfiguration(NodeConfiguration):

    @dataclasses.dataclass
    class Connector:
        bus: int
        address: int

    sensor_name: str
    frequency: int
    mode: str
    display_fragment_frequency: int
    connectors: List[Connector]


class ToFNode(Node):
    """
    This class implements the communication logic with a Time-of-Flight sensor on the i2c bus.
    It publishes both range measurements and display fragments to show on an LCD screen.

    NOTE: Out-of-range readings do not stop the stream of messages. Instead, a message with a
          range value well outside the domain [min_range, max_range] will be published.
          Such value is sensor-specific and at the time of this writing, this number is 8.0.
          As per the official ROS's documentation on the Range message
          (https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Range.html),
          "values < range_min or > range_max should be discarded".
          So, it is the consumer's responsibility to handle out-of-range readings.

    """

    def __init__(self, sensor_name: str, config: str):
        super().__init__(
            name=f"tof-{sensor_name}",
            kind=NodeType.DRIVER,
            description="Time-of-Flight sensor driver",
        )
        self._sensor_name: str = sensor_name

        # load configuration
        self.configuration: ToFNodeConfiguration = ToFNodeConfiguration.from_name(self.package, config)

        # load accuracy profile
        self._accuracy: ToFAccuracy = ToFAccuracy.from_string(self.configuration.mode)

        # compute frequency
        self._frequency: int = self.configuration.frequency
        max_frequency = min(self.configuration.frequency, int(1.0 / self._accuracy.timing_budget))
        if self.configuration.frequency > max_frequency:
            self.logger.warning(
                f"Frequency of {self.configuration.frequency}Hz not supported. The selected mode "
                f"{self.configuration.mode} has a timing budget of {self._accuracy.timing_budget}s, "
                f"which yields a maximum frequency of {max_frequency}Hz."
            )
            self._frequency = max_frequency
        self.logger.info(f"Frequency set to {self._frequency}Hz.")

        # frame
        self._frame_id: str = f"{self._robot_name}/tof/{self._sensor_name}"

        # create a ToF sensor handler
        self._sensor: Optional[ToFDriver] = self._find_sensor()
        if not self._sensor:
            self.logger.error(f"No ToF device found. These connectors were tested:\n{self.configuration.connectors}\n")
            exit(1)

        # screen renderer
        self._renderer: Optional[ToFSensorFragmentRenderer] = None

    def _find_sensor(self) -> Optional[ToFDriver]:
        if get_robot_hardware() != RobotHardware.VIRTUAL:
            if not self.configuration.connectors:
                print("No i2c connectors specified in the configuration")

            for connector in self.configuration.connectors:
                conn: str = "[bus:{bus}](0x{address:02X})".format(**dataclasses.asdict(connector))
                self.logger.info(f"Trying to open device on connector {conn}")

                bus_dev = f"/dev/i2c-{connector.bus}"
                if not os.path.exists(bus_dev):
                    self.logger.warning(f"No devices found on connector {conn}, the bus does NOT exist")
                    continue

                sensor = ToFDriver(
                    accuracy=self._accuracy,
                    i2c_bus=connector.bus,
                    i2c_address=connector.address,
                    name=self._sensor_name
                )
                sensor.setup()

                sensor.start()

                time.sleep(1)
                if sensor.get_distance() < 0:
                    self.logger.warning(f"No devices found on connector {conn}, but the bus exists")
                    continue
                self.logger.info(f"Device found on connector {conn}")

                time.sleep(2)

                return sensor
        else:
            sensor = ToFDriver(accuracy=self._accuracy, name=self._sensor_name)
            sensor.setup()
            sensor.start()
            return sensor

    async def worker(self):
        await self.dtps_init(self.configuration)
        # create sensor queues
        range_queue = await (self.context / "out" / "range").queue_create()
        info_queue = await (self.context / "out" / "info").queue_create()
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "sensors" / "time-of-flight" / self._sensor_name / "range").expose(range_queue)
        await (self.switchboard / "sensors" / "time-of-flight" / self._sensor_name / "info").expose(info_queue)
        # publish info about the sensor
        msg = RangeFinder(
            fov=self._accuracy.fov,
            minimum=self._accuracy.min_range,
            maximum=self._accuracy.max_range,
        )
        await info_queue.publish(msg.to_rawdata())
        # read and publish
        dt: float = 1.0 / self._frequency
        while not self.is_shutdown:
            # detect range
            range_mm: float = self._sensor.get_distance()
            range_m: float = range_mm / 1000
            data: float | None = range_m if range_m <= self._accuracy.max_range else None
            # pack observation into a message
            msg = Range(
                header=Header(frame=self._frame_id),
                data=data,
            )
            await range_queue.publish(msg.to_rawdata())
            # update display
            if self._renderer is not None:
                self._renderer.update(range_mm)
            # wait
            await asyncio.sleep(dt)

    @sidecar
    async def worker_display(self):
        # wait for switchboard
        await self.switchboard_ready.wait()
        # wait for display
        display: DTPSContext = await (self.switchboard / "actuator" / "display" / "fragments").until_ready()
        publisher: PublisherInterface = await display.publisher()
        # create screen renderer
        self._renderer = ToFSensorFragmentRenderer(
            self._sensor_name,
            self._accuracy,
            self.configuration.display_fragment_frequency,
            publisher
        )
        # lauch display renderer
        await self._renderer.worker()

    def on_shutdown(self):
        # noinspection PyBroadException
        try:
            self._sensor.stop()
        except BaseException:
            pass


class ToFSensorFragmentRenderer(MonoImageFragmentRenderer):

    def __init__(self, name: str, accuracy: ToFAccuracy, frequency: float, publisher: PublisherInterface):
        super(ToFSensorFragmentRenderer, self).__init__(
            f"__tof_{name}__",
            page=PAGE_TOF,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
            callback=self.publish,
            frequency=frequency
        )
        self._accuracy = accuracy
        self._publisher: PublisherInterface = publisher
        name = name.replace("_", " ").title()
        self._range_mm: float | None = None
        self._title_h = 12
        self._title = monospace_screen((self._title_h, self.roi.w), f"ToF / {name}:", scale="vfill")

    def update(self, range_mm: float):
        self._range_mm = range_mm

    async def step(self):
        if self._range_mm is None:
            return
        # render title
        pretty_range = (
            f" {(self._range_mm / 10):.1f}cm "
            if (self._range_mm / 1000) < self._accuracy.max_range
            else "Out-Of-Range"
        )
        # render reading
        reading = monospace_screen(
            (self.roi.h - self._title_h, self.roi.w), pretty_range, scale="hfill", align="center"
        )
        # update buffer
        self._buffer[: self._title_h, :] = self._title
        self._buffer[self._title_h:, :] = reading

    async def publish(self, _):
        await self._publisher.publish(DisplayFragments(
            fragments=self.fragments
        ).to_rawdata())


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--sensor-name", type=str, required=True, help="Name of the sensor")
    parser.add_argument("--config", type=str, required=True, help="Name of the configuration")
    args: argparse.Namespace = parser.parse_args()
    # create node
    node: ToFNode = ToFNode(sensor_name=args.sensor_name, config=args.config)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
