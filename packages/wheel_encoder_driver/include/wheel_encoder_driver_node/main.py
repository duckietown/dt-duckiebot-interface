#!/usr/bin/env python3

import dataclasses
import asyncio
import math
from typing import Optional

import argparse
import numpy as np

from dt_node_utils.config import NodeConfiguration

from dtps_http import RawData

from dt_class_utils import DTReminder
from dt_node_utils import NodeType
from dt_node_utils.node import Node
from wheel_encoder_driver.wheel_encoder_abs import WheelEncoderDriverAbs
from pytransform3d import transformations, rotations

from wheel_encoder_driver import WheelEncoderDriver


@dataclasses.dataclass
class WheelEncoderNodeConfiguration(NodeConfiguration):
    gpio: Optional[int]
    resolution: int
    publish_frequency: float
    renderer_frequency: float


class WheelEncoderNode(Node):
    """
    This class implements the communication logic with a wheel encoder sensor via GPIO pins.
    """

    def __init__(self, side: str, config: str):
        super().__init__(
            name=f"wheel-encoder-{side}",
            kind=NodeType.DRIVER,
            description="Wheel encoder sensor driver",
        )
        self._side: str = side

        # load configuration
        self.configuration: WheelEncoderNodeConfiguration = (WheelEncoderNodeConfiguration.
                                                             from_name(self.package, config))

        # frames
        self._motor_frame_id: str = f"{self._robot_name}/motor/{self._side}"
        self._wheel_frame_id: str = f"{self._robot_name}/motor/{self._side}/wheel"

        # create a WheelEncoder sensor handler
        self._sensor: WheelEncoderDriverAbs = WheelEncoderDriver(self._side, self.configuration.gpio)

        # create screen renderer
        self._renderer_reminder: DTReminder = DTReminder(frequency=self.configuration.renderer_frequency)
        # self._renderer = WheelEncoderSensorFragmentRenderer(self._side, self._accuracy)

    @property
    def period(self) -> float:
        return 1. / self.configuration.publish_frequency

    async def worker(self):
        await self.dtps_init(self.configuration)
        # create sensor queue
        queue = await (self.context / "out" / "ticks").queue_create()
        # TODO: we need to make this global and rethink how (origin, target, transform) TFs are stored in the same queue
        tf_queue = await (self.context / "out" / "tf").queue_create()
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "sensors" / "wheel-encoder" / self._side).expose(queue)
        # read and publish
        while not self.is_shutdown:
            # pack observation into a message
            data = {
                'data': self._sensor.ticks,
            }
            # publish readings
            rdata = RawData.cbor_from_native_object(data)
            await queue.publish(rdata)

            # publish frame updates
            angle = (float(self._sensor.ticks) / float(self.configuration.resolution)) * 2 * math.pi
            quat: np.ndarray = rotations.quaternion_from_euler([0, angle, 0], 0, 1, 2, False)
            tf: np.ndarray = transformations.transform_from_pq([0, 0, 0, *quat])
            # TODO: we should make a common class for this
            data = {
                "origin": self._motor_frame_id,
                "target": self._wheel_frame_id,
                "transform": tf.tolist(),
            }
            # TODO: we should wrap this into a TF class
            rdata = RawData.cbor_from_native_object(data)
            await tf_queue.publish(rdata)

            # publish display rendering (if it is a good time to do so)
            if self._renderer_reminder.is_time():
                # TODO: implement this
                # self._renderer.update(distance_mm)
                # msg = self._renderer.as_msg()
                # self._display_pub.publish(msg)
                pass

            # ---
            await asyncio.sleep(self.period)

    def on_shutdown(self):
        # noinspection PyBroadException
        try:
            self._sensor.release()
        except BaseException:
            pass


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--side", type=str, required=True, help="Side (left, or right)")
    parser.add_argument("--config", type=str, required=True, help="Name of the configuration")
    args: argparse.Namespace = parser.parse_args()
    # create node
    node: WheelEncoderNode = WheelEncoderNode(side=args.side, config=args.config)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()