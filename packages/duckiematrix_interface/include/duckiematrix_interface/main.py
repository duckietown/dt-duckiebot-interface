#!/usr/bin/env python3

import asyncio

from dt_node_utils import NodeType
from dt_node_utils.node import Node
from hil_support.hil import HardwareInTheLoopSupport, HardwareInTheLoopSide


class DuckiematrixInterface(Node, HardwareInTheLoopSupport):
    """
    This class exposes topics from the Duckiematrix to the robot interface.
    
    Currently supported topics:
    - `{matrix_key}/pose/pose` (Transformation): the pose of the robot in the world frame.
        Gets remapped to `{ROBOT_NAME}/pose`.
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
        # create pose queues
        gt_pose_context = self.context / "out" / "pose"
        ground_truth_pose_queue = await (gt_pose_context).queue_create()

        # create twist queues
        gt_twist_context = self.context / "out" / "twist"
        ground_truth_twist_queue = await (gt_twist_context).queue_create()

        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "pose").expose(ground_truth_pose_queue)
        await (self.switchboard / "twist").expose(ground_truth_twist_queue)

        # Remap the pose topic from {matrix_key}/pose/pose to {ROBOT_NAME}/pose
        await self.init_hil_support(
            self.context,
            src=None,
            src_path=['pose'],
            dst=self.context,
            dst_path=["out"],
            subpaths=["pose"],
            side=HardwareInTheLoopSide.SOURCE,
            # TODO: use transformations to set the frame in the message
        )

        # Remap the twist topic from {matrix_key}/twist/twist to {ROBOT_NAME}/twist
        await self.init_hil_support(
            self.context,
            src=None,
            src_path=['twist'],
            dst=self.context,
            dst_path=["out"],
            subpaths=["twist"],
            side=HardwareInTheLoopSide.SOURCE,
            # TODO: use transformations to set the frame in the message
        )

        while not self.is_shutdown:
            # do nothing if HIL is active
            if self.hil_is_active:
                await asyncio.sleep(1.0)
                continue

def main():
    node: DuckiematrixInterface = DuckiematrixInterface()
    node.spin()


if __name__ == "__main__":
    main()
