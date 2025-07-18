#!/usr/bin/env python3
"""The Control Mapper node."""

import time

from dt_node_utils import NodeType
from dt_node_utils.node import Node
from duckietown_messages.standard.boolean import Boolean
from duckietown_messages.standard.header import Header


class ControlMapperNode(Node):
    """Control Mapper node."""

    def __init__(self) -> None:
        """Initialize Control Mapper node."""
        super().__init__(
            "control_mapper",
            NodeType.MAPPING,
            "A control mapper.",
        )
        self.loginfo("Initialized.")

    async def worker(self) -> None:
        """Worker."""
        self.loginfo("Initializing DTPS...")
        await self.dtps_init()
        self.loginfo("DTPS initialized.")
        self.loginfo("Creating queues...")
        autopilot_queue = await (self.context / "autopilot").queue_create()
        self.loginfo("Queues created.")
        self.loginfo("Exposing node to switchboard...")
        await self.dtps_expose()
        self.loginfo("Node exposed to switchboard.")
        self.loginfo("Publishing initial states...")
        timestamp = time.time()
        header = Header(timestamp=timestamp)
        autopilot_message = Boolean(header=header, data=False).to_rawdata()
        await autopilot_queue.publish(autopilot_message)
        self.loginfo("Initial states published.")
        self.loginfo("Running...")
        await self.join()
        self.loginfo("Shutting down...")


if __name__ == "__main__":
    node = ControlMapperNode()
    node.spin()
