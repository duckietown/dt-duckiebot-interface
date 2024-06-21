#!/usr/bin/env python3

import argparse
import asyncio
from typing import Tuple

from camera_driver import CameraNodeAbs


class CameraNode(CameraNodeAbs):
    """
    Handles the imagery on a Virtual robot.
    """

    def __init__(self, config: str, sensor_name: str):
        # Initialize the DTROS parent class
        super(CameraNode, self).__init__(config, sensor_name)
        # ---
        self.loginfo("[CameraNode]: Initialized.")

    def get_hil_bridged_topic(self) -> Tuple[str, ...]:
        return "sensor", "camera", self.sensor_name

    async def worker(self):
        """
        Image capture procedure.

        Captures frames from the camera and publishes them.
        """
        # init queues
        await self.dtps_init_queues()
        # keep loop alive
        while not self.is_shutdown:
            await asyncio.sleep(1.0)
        self.loginfo("Camera worker stopped.")

    def setup(self):
        pass

    def on_shutdown(self):
        self.deinit_hil_support()


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--sensor-name", type=str, required=True, help="Name of the sensor")
    parser.add_argument("--config", type=str, required=True, help="Name of the configuration")
    args: argparse.Namespace = parser.parse_args()
    # create node
    node: CameraNode = CameraNode(config=args.config, sensor_name=args.sensor_name)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
