#!/usr/bin/env python3
import asyncio
import dataclasses
from typing import Optional, List

import argparse

from dt_class_utils import DTReminder
from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.decorators import sidecar
from dt_node_utils.node import Node
from duckietown_messages.sensors.angular_velocities import AngularVelocities
from duckietown_messages.sensors.linear_accelerations import LinearAccelerations
from duckietown_messages.sensors.temperature import Temperature
from duckietown_messages.standard.dictionary import Dictionary
from imu_driver.exceptions import DeviceNotFound
from imu_driver.mpu6050 import CalibratedMPU6050
from imu_driver.types import I2CConnector


# from display_renderer import (
#     DisplayROI,
#     PAGE_TOF,
#     REGION_BODY,
#     MonoImageFragmentRenderer,
# )
# from display_renderer.text import monospace_screen


@dataclasses.dataclass
class IMUNodeConfiguration(NodeConfiguration):
    frequency: float
    display_fragment_frequency: float
    connectors: List[I2CConnector]


class IMUNode(Node):
    """
    This class implements the communication logic with an IMU sensor on the i2c bus.
    It publishes both measurements and display fragments to show on an LCD screen.

    """

    def __init__(self, config: str):
        node_name: str = "imu-driver"
        super().__init__(
            name=node_name,
            kind=NodeType.DRIVER,
            description="IMU (Inertia Measurement Unit) sensor driver",
        )

        # load configuration
        self.configuration: IMUNodeConfiguration = IMUNodeConfiguration.from_name(self.package, node_name, config)

        # frame
        self._frame_id: str = f"{self._robot_name}/imu"

        # create a IMU sensor handler
        try:
            self._sensor: Optional[CalibratedMPU6050] = CalibratedMPU6050(
                self.configuration.connectors, self.context, self.logger
            )
        except DeviceNotFound:
            self.logger.error(f"No IMU device found. These connectors were tested:\n{self.configuration.connectors}\n")
            exit(1)

        # create screen renderer
        # self._renderer = IMUSensorFragmentRenderer(self._accuracy)

        # create reminders
        self._fragment_reminder = DTReminder(frequency=self.configuration.display_fragment_frequency)

    async def worker(self):
        await self.dtps_init(self.configuration)
        # create sensor queues
        accelerations_queue = await (self.context / "out" / "acceleration" / "linear").queue_create()
        velocities_queue = await (self.context / "out" / "velocity" / "angular").queue_create()
        temperature_queue = await (self.context / "out" / "temperature").queue_create()
        raw_queue = await (self.context / "out" / "raw").queue_create()
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "sensors" / "imu" / "accelerometer").expose(accelerations_queue)
        await (self.switchboard / "sensors" / "imu" / "gyroscope").expose(velocities_queue)
        await (self.switchboard / "sensors" / "imu" / "temperature").expose(temperature_queue)
        await (self.switchboard / "sensors" / "imu" / "raw").expose(raw_queue)
        # read and publish
        dt: float = 1.0 / self.configuration.frequency
        while not self.is_shutdown:
            try:
                # read data from the sensors and pack into messages
                acc: List[float] = self._sensor.linear_accelerations
                accelerations: LinearAccelerations = LinearAccelerations(x=acc[0], y=acc[1], z=acc[2])
                vel: List[float] = self._sensor.angular_velocities
                velocities: AngularVelocities = AngularVelocities(x=vel[0], y=vel[1], z=vel[2])
                temp: float = self._sensor.temperature
                temperature: Temperature = Temperature(data=temp)
            except Exception as e:
                self.logwarn(f"IMU Comm Loss: {e}")
            else:
                # pack raw data
                raw: Dictionary = Dictionary(data={
                    "linear_accelerations": acc,
                    "angular_velocities": vel,
                    "temperature": temp,
                })
                # publish
                await accelerations_queue.publish(accelerations.to_rawdata())
                await velocities_queue.publish(velocities.to_rawdata())
                await temperature_queue.publish(temperature.to_rawdata())
                await raw_queue.publish(raw.to_rawdata())
            finally:
                # wait
                await asyncio.sleep(dt)

    @sidecar
    async def display_worker(self):
        dt: float = 1.0 / self.configuration.display_fragment_frequency
        while not self.is_shutdown:
            # TODO: implement display rendering
            pass
            await asyncio.sleep(dt)


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True, help="Name of the configuration")
    args: argparse.Namespace = parser.parse_args()
    # create node
    node: IMUNode = IMUNode(config=args.config)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
