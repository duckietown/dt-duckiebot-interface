#!/usr/bin/env python3

import asyncio
import dataclasses
import os
import time
from typing import Optional, List

import argparse
from dt_robot_utils.constants import RobotType
from dt_robot_utils.robot import get_robot_type
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
from hil_support.hil import HardwareInTheLoopSupport, HardwareInTheLoopSide
from tof_driver import ToFDriver, ToFAccuracy


@dataclasses.dataclass
class ToFNodeConfiguration(NodeConfiguration):

    @dataclasses.dataclass
    class Connector:
        bus: int
        address: int

    sensor_name: str
    sensor_model: str
    frequency: int
    mode: str
    display_fragment_frequency: int
    connectors: List[Connector]
    # milliseconds. None keeps the mode default
    timing_budget: Optional[int] = None
    # milliseconds. None derives it from `frequency`
    inter_measurement_period: Optional[int] = None


class ToFNode(Node, HardwareInTheLoopSupport):
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
        node_name = f"tof_driver_{sensor_name}"
        super().__init__(
            name=node_name,
            kind=NodeType.DRIVER,
            description="Time-of-Flight sensor driver",
        )
        HardwareInTheLoopSupport.__init__(self)
        self.sensor_name: str = sensor_name

        # load configuration
        self.configuration: ToFNodeConfiguration = ToFNodeConfiguration.from_name(self.package, node_name, config)

        # load accuracy profile
        self._accuracy: ToFAccuracy = ToFAccuracy.from_string(
            self.configuration.mode,
            self.configuration.sensor_model
            )

        # chip-level knobs, when the configuration asks for them
        if self.configuration.timing_budget is not None:
            self._accuracy.timing_budget = self.configuration.timing_budget / 1000.0
        if self.configuration.inter_measurement_period is not None:
            self._accuracy.inter_measurement_period = (
                self.configuration.inter_measurement_period / 1000.0
            )
        else:
            # measure as often as the publish rate needs, never faster than one takes
            self._accuracy.inter_measurement_period = max(
                1.0 / self.configuration.frequency, self._accuracy.timing_budget
            )

        # fail here rather than inside the bus probe, where it would read as missing hardware
        self._accuracy.validate(self.configuration.sensor_model)

        # the inter-measurement period is the rate ceiling, not the timing budget
        self._frequency: float = self.configuration.frequency
        max_frequency: float = self._accuracy.max_frequency

        assert max_frequency > 0, "The timing budget is too low."

        if self._frequency > max_frequency:
            self.logger.warning(
                f"Frequency of {self.configuration.frequency}Hz not supported. Mode "
                f"{self.configuration.mode} with a timing budget of "
                f"{self._accuracy.timing_budget_ms}ms and an inter-measurement period of "
                f"{self._accuracy.inter_measurement_period_ms}ms yields a maximum "
                f"frequency of {max_frequency:.1f}Hz."
            )
            self._frequency = max_frequency
        self.logger.info(
            f"Frequency set to {self._frequency:.1f}Hz "
            f"(timing budget {self._accuracy.timing_budget_ms}ms, "
            f"inter-measurement period {self._accuracy.inter_measurement_period_ms}ms)."
        )

        # frame
        self.frame_id: str = f"{self._robot_name}/tof/{self.sensor_name}"

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
                sensor: Optional[ToFDriver] = None
                try:
                    sensor = ToFDriver(
                        accuracy=self._accuracy,
                        i2c_bus=connector.bus,
                        i2c_address=connector.address,
                        name=self.sensor_name
                    )
                    sensor.setup()

                    sensor.start()

                    time.sleep(1)
                    dist = sensor.get_distance()
                    if dist is None or dist < 0:
                        self.logger.warning(f"No devices found on connector {conn}, but the bus exists")
                        if getattr(sensor, "_sensor", None) is not None:
                            try:
                                sensor.stop()
                            except Exception as e:
                                self.logger.debug(f"Cleanup failed stopping sensor on connector {conn}: {e}")
                        continue
                    self.logger.info(f"Device found on connector {conn}")

                    time.sleep(2)

                    return sensor
                except ValueError as e:
                    # concise message for device-probe failures
                    if "No I2C device" in str(e):
                        self.logger.warning(f"No I2C device on connector {conn}")
                        continue
                    self.logger.exception(f"Exception while trying connector {conn}")
                    if getattr(sensor, "_sensor", None) is not None:
                        try:
                            sensor.stop()
                        except Exception as e2:
                            self.logger.debug(f"Cleanup failed stopping sensor on connector {conn}: {e2}")
                    continue
                except (OSError, IOError, RuntimeError) as e:
                    self.logger.warning(f"I2C error on connector {conn}: {e}")
                    if getattr(sensor, "_sensor", None) is not None:
                        try:
                            sensor.stop()
                        except Exception as e2:
                            self.logger.debug(f"Cleanup failed stopping sensor on connector {conn}: {e2}")
                    continue
                except Exception:
                    self.logger.exception(f"Unexpected exception while trying connector {conn}")
                    if getattr(sensor, "_sensor", None) is not None:
                        try:
                            sensor.stop()
                        except Exception as e2:
                            self.logger.debug(f"Cleanup failed stopping sensor on connector {conn}: {e2}")
                    continue
        else:
            sensor = ToFDriver(accuracy=self._accuracy, name=self.sensor_name)
            sensor.setup()
            sensor.start()
            return sensor

    async def worker(self):
        await self.dtps_init(self.configuration)
        # create sensor queues
        range_queue = await (self.context / "out" / "range").queue_create()
        info_queue = await (self.context / "out" / "info").queue_create()
        # create publishers
        range_publisher = await range_queue.publisher()
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "sensor" / "time_of_flight" / self.sensor_name / "range").expose(range_queue)
        await (self.switchboard / "sensor" / "time_of_flight" / self.sensor_name / "info").expose(info_queue)
        # initialize HIL support
        await self.init_hil_support(
            self.context,
            # source (this is the dynamic side, duckiematrix or nothing)
            src=None,
            src_path=["sensor", "time_of_flight", self.sensor_name],
            # destination (this is us, static)
            dst=self.context,
            dst_path=["out"],
            # paths to connect when a remote is set
            subpaths=["range"],
            # which side is the re-pluggable one
            side=HardwareInTheLoopSide.SOURCE,
            # TODO: use transformations to set the frame in the message
        )
        # publish info about the sensor
        timestamp = time.time()
        header = Header(timestamp=timestamp)
        msg = RangeFinder(
            # -- base
            header=header,
            # -- sensor
            name=self.sensor_name,
            type="time-of-flight",
            simulated=False,
            description="Time-of-Flight range sensor",
            frame_id=self.frame_id,
            frequency=self._frequency,
            maker="STMicroelectronics",
            model=self.configuration.sensor_model,
            # -- RangeFinder
            fov=self._accuracy.fov,
            minimum=self._accuracy.min_range,
            maximum=self._accuracy.max_range,
        )
        await info_queue.publish(msg.to_rawdata())
        # read and publish
        dt: float = 1.0 / self._frequency
        # schedule against the monotonic clock, so an NTP step does not stall or rush the
        # loop. Message timestamps stay on the wall clock, consumers correlate against it
        loop: asyncio.AbstractEventLoop = asyncio.get_running_loop()
        next_slot: float = loop.time()
        while not self.is_shutdown:
            # do nothing if HIL is active
            if self.hil_is_active:
                await asyncio.sleep(1.0)
                next_slot = loop.time()
                continue
            if self._sensor is None:
                self.logger.error("The sensor is not responding.")
                await asyncio.sleep(dt)
                continue
            # ---
            try:
                # skip repeats: without this the result register hands out the previous
                # measurement again, which looks like data but carries nothing new. This
                # is an I2C read too, so it shares the bus-error handler below
                if not await self._wait_for_measurement(dt):
                    self.logger.debug("No new measurement within one period, skipping.")
                    continue
                # detect range
                range_mm: Optional[float] = self._sensor.get_distance()
            except OSError:
                self.logger.error("I2C error. The sensor is not responding.")
                await asyncio.sleep(dt)
                continue
            # ---
            data: float | None = None
            # convert to meters, check for out-of-range
            if range_mm is not None:
                range_m: float = range_mm / 1000
                data = range_m if range_m <= self._accuracy.max_range else None
            else:
                self.logger.error("The sensor is not responding.")
                await asyncio.sleep(1.0)
                continue

            # pack observation into a message
            timestamp = time.time()
            header = Header(frame=self.frame_id, timestamp=timestamp)
            msg = Range(
                header=header,
                data=data,
            )
            await range_publisher.publish(msg.to_rawdata())
            # update display
            if self._renderer is not None:
                self._renderer.update(range_mm)
            # sleep to a deadline, not for a fixed delay, or the read time eats the rate
            next_slot = max(next_slot + dt, loop.time())
            await asyncio.sleep(next_slot - loop.time())

    async def _wait_for_measurement(self, timeout: float) -> bool:
        """Wait for a measurement the sensor has not handed out yet. Each check is an
        I2C read, so poll in proportion to how long a measurement takes rather than as
        fast as possible.
        """
        loop: asyncio.AbstractEventLoop = asyncio.get_running_loop()
        deadline: float = loop.time() + timeout
        poll: float = min(max(self._accuracy.inter_measurement_period / 20, 0.001), 0.010)
        while not self._sensor.data_ready:
            if loop.time() >= deadline:
                return False
            await asyncio.sleep(poll)
        return True

    @sidecar
    async def worker_display(self):
        # wait for switchboard
        await self.switchboard_ready.wait()

        if get_robot_type() != RobotType.DUCKIEBOT or get_robot_hardware() == RobotHardware.VIRTUAL:
            # skip display renderer if not a Duckiebot
            return

        # wait for display
        display: DTPSContext = await ((self.switchboard / "actuator" / "display" / "interaction_plate" / "fragments")
                                      .until_ready())
        publisher: PublisherInterface = await display.publisher()
        # create screen renderer
        self._renderer = ToFSensorFragmentRenderer(
            self.sensor_name,
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
        timestamp = time.time()
        header = Header(timestamp=timestamp)
        await self._publisher.publish(DisplayFragments(
            header=header,
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
