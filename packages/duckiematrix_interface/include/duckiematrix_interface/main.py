#!/usr/bin/env python3

import asyncio

from dt_node_utils import NodeType
from dt_node_utils.node import Node
from hil_support.hil import HardwareInTheLoopSupport, HardwareInTheLoopSide


class DuckiematrixInterface(Node, HardwareInTheLoopSupport):
    """
    This class exposes topics from the Duckiematrix to the robot interface.

    Currently supported topics:
    - `{matrix_key}/state/pose` (Transformation): the pose of the robot in the world frame.
        Gets remapped to `{ROBOT_NAME}/state/pose`.
    - `{matrix_key}/state/twist` (Twist): the linear and angular velocity of the robot gets
        published as a Twist message. Gets remapped to `{ROBOT_NAME}/state/twist`.
    """

    def __init__(self):
        node_name = "duckiematrix_interface"
        super().__init__(
            name=node_name,
            kind=NodeType.GENERIC,
            description="Driver exposing topics from the Duckiematrix",
        )
        HardwareInTheLoopSupport.__init__(self)

    async def worker(self):
        await self.dtps_init()
        # create queues
        pose_queue = await (self.context / "out" / "state" / "pose").queue_create()
        twist_queue = await (self.context / "out" / "state" / "twist").queue_create()
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "state" / "pose").expose(pose_queue)
        await (self.switchboard / "state" / "twist").expose(twist_queue)
        # initialize HIL support
        await self.init_hil_support(
            self.context,
            # source (this is the dynamic side, duckiematrix or nothing)
            src=None,
            src_path=["state"],
            # destination (this is us, static)
            dst=self.context,
            dst_path=["out", "state"],
            # paths to connect when a remote is set
            subpaths=["pose", "twist"],
            # which side is the re-pluggable one
            side=HardwareInTheLoopSide.SOURCE,
            # TODO: use transformations to set the frame in the message
        )
        while not self.is_shutdown:
            await asyncio.sleep(1.0)

def main():
    node: DuckiematrixInterface = DuckiematrixInterface()
    node.spin()


if __name__ == "__main__":
    main()
