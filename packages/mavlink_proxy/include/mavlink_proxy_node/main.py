#!/usr/bin/env python3
"""
MAVLink Proxy Node

Bridges MAVLink TCP connection with DTPS switchboard.
Exposes MAVLink streams to the duckiematrix engine via switchboard.

This node:
1. Connects to PX4 SITL via TCP
2. Exposes MAVLink TX/RX queues to the switchboard
3. Handles bidirectional MAVLink forwarding

Architecture:
  PX4 SITL (tcp:4560) <--> MAVLinkProxyNode <--> Switchboard <--> Duckiematrix Engine
"""

import argparse
import asyncio
import dataclasses
import socket
from typing import Optional

from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.node import Node
from dtps_http import RawData
from hil_support.hil import HardwareInTheLoopSupport, HardwareInTheLoopSide


@dataclasses.dataclass
class MAVLinkProxyNodeConfiguration(NodeConfiguration):
    """Configuration for MAVLink proxy node."""
    
    # Default to localhost - works when container runs with --net=host
    # or when PX4 SITL is in same container
    px4_host: str = "localhost"
    px4_port: int = 4560
    buffer_size: int = 4096
    # Connection retry settings
    connect_retry_interval: float = 1.0  # seconds between retries
    connect_timeout: float = 5.0  # socket connect timeout


class MAVLinkProxyNode(Node):
    """
    MAVLink Proxy Node.
    
    Bridges MAVLink TCP connection with DTPS switchboard following
    the same pattern as IMU and LED drivers.
    """
    
    def __init__(self, config: str, proxy_name: str = "mavlink"):
        node_name: str = f"mavlink_proxy_{proxy_name}"
        super().__init__(
            name=node_name,
            kind=NodeType.DRIVER,
            description="MAVLink TCP-DTPS proxy",
        )
        self.proxy_name: str = proxy_name
        
        # load configuration
        self.configuration: MAVLinkProxyNodeConfiguration = (
            MAVLinkProxyNodeConfiguration.from_name(self.package, node_name, config)
        )
        
        # TCP server for PX4 connections
        self._server: Optional[asyncio.Server] = None
        self._px4_reader: Optional[asyncio.StreamReader] = None
        self._px4_writer: Optional[asyncio.StreamWriter] = None
        self._connected: bool = False
        
        # HIL support for bidirectional bridging
        self._hil_tx: HardwareInTheLoopSupport = HardwareInTheLoopSupport()
        self._hil_rx: HardwareInTheLoopSupport = HardwareInTheLoopSupport()
    
    async def _handle_px4_connection(self, reader: asyncio.StreamReader, 
                                     writer: asyncio.StreamWriter):
        """
        Handle incoming PX4 SITL connection.
        
        PX4 SITL connects to us as a client (like MAVProxy pattern).
        """
        addr = writer.get_extra_info('peername')
        self.loginfo(f"✓ PX4 SITL connected from {addr[0]}:{addr[1]}")
        
        # Store current connection
        self._px4_reader = reader
        self._px4_writer = writer
        self._connected = True
        
        try:
            # Keep connection alive until it closes
            while not self.is_shutdown:
                # Just check if connection is still alive
                if reader.at_eof():
                    break
                await asyncio.sleep(0.1)
        except Exception as e:
            self.logwarn(f"PX4 connection error: {e}")
        finally:
            self._disconnect()
    
    def _disconnect(self):
        """Disconnect from PX4."""
        if self._connected:
            self.loginfo("PX4 SITL disconnected")
        
        if self._px4_writer:
            try:
                self._px4_writer.close()
            except:
                pass
        self._px4_reader = None
        self._px4_writer = None
        self._connected = False
    
    async def cb_mavlink_tx(self, data: RawData):
        """
        Callback for MAVLink TX (data coming FROM engine TO PX4).
        
        This receives MAVLink telemetry from the duckiematrix engine
        (via HIL bridge) and forwards it to PX4 SITL via TCP.
        """
        if not self._connected or not self._px4_writer:
            return
        
        try:
            content = data.content if hasattr(data, 'content') else data
            if isinstance(content, bytes) and len(content) > 0:
                self._px4_writer.write(content)
                await self._px4_writer.drain()
                self.logdebug(f"Forwarded {len(content)} bytes from engine to PX4")
        except Exception as e:
            self.logwarn(f"Error forwarding engine data to PX4: {e}")
            self._disconnect()
    
    async def cb_mavlink_rx(self, data: RawData):
        """
        Callback for MAVLink RX (data coming FROM switchboard TO engine).
        
        This receives MAVLink commands from the switchboard
        and they get forwarded to engine via HIL bridge.
        """
        # Data is automatically forwarded to engine by HIL RX bridge
        # This callback is here for potential logging/monitoring
        pass
    
    async def _read_from_px4_task(self, tx_publisher):
        """
        Background task to read from PX4 and publish to TX queue.
        
        This continuously reads MAVLink data from PX4 SITL and publishes
        it to the switchboard TX queue so the engine can consume it.
        """
        messages_read = 0
        
        while not self.is_shutdown:
            if not self._connected or not self._px4_reader:
                await asyncio.sleep(0.5)
                continue
            
            try:
                # Async read from StreamReader
                data = await self._px4_reader.read(self.configuration.buffer_size)
                if data and len(data) > 0:
                    messages_read += 1
                    # Wrap in RawData and publish
                    raw_data = RawData(
                        content=data,
                        content_type="application/octet-stream"
                    )
                    await tx_publisher.publish(raw_data)
                    
                    if messages_read == 1:
                        self.loginfo("Started reading MAVLink data from PX4")
                elif self._px4_reader.at_eof():
                    # Connection closed
                    self.loginfo("PX4 closed connection")
                    self._disconnect()
                    await asyncio.sleep(0.5)
            except Exception as e:
                self.logwarn(f"Error reading from PX4: {e}")
                self._disconnect()
                await asyncio.sleep(0.5)
    
    async def worker(self):
        """Main worker loop following the driver pattern."""
        await self.dtps_init(self.configuration)
        
        # Create MAVLink queues (following sensor/actuator pattern)
        # TX: Data flowing FROM PX4 TO engine (sensor-like)
        tx_queue = await (self.context / "out" / "tx").queue_create()
        # RX: Data flowing FROM engine TO PX4 (actuator-like)
        rx_queue = await (self.context / "in" / "rx").queue_create()
        
        # Create publisher for TX (we publish data we read from PX4)
        tx_publisher = await tx_queue.publisher()
        
        # Subscribe to TX (we receive telemetry from engine via HIL and forward to PX4)
        await tx_queue.subscribe(self.cb_mavlink_tx)
        
        # Subscribe to RX (we receive data from switchboard, HIL forwards to engine)
        await rx_queue.subscribe(self.cb_mavlink_rx)
        
        # Expose node to the switchboard
        await self.dtps_expose()
        
        # Expose queues to the switchboard
        # The engine's robot_connector will subscribe to these paths
        await (
            self.switchboard / "sensor" / "mavlink" / self.proxy_name / "tx"
        ).expose(tx_queue)
        await (
            self.switchboard / "actuator" / "mavlink" / self.proxy_name / "rx"
        ).expose(rx_queue)
        
        # Initialize HIL support for TX direction (engine -> node)
        # Engine publishes sensor data that we receive
        await self._hil_tx.init_hil_support(
            node=self.context,
            src=None,  # Engine side (dynamic source)
            src_path=['sensor', 'mavlink','mavlink'],
            dst=self.context,  # This node (static destination)
            dst_path=['out'],
            subpaths=['tx'],
            side=HardwareInTheLoopSide.SOURCE,  # Engine is the dynamic source
        )
        
        # Initialize HIL support for RX direction (node -> engine)
        # We receive from switchboard and forward to engine
        await self._hil_rx.init_hil_support(
            node=self.context,
            src=self.context,  # This node (static source)
            src_path=['in'],
            dst=None,  # Engine side (dynamic destination)
            dst_path=['actuator', 'mavlink', 'mavlink'],
            subpaths=['rx'],
            side=HardwareInTheLoopSide.DESTINATION,  # Engine is the dynamic destination
        )
        
        self.loginfo(
            f"Exposed MAVLink queues to switchboard:\n"
            f"  TX (sensor): /sensor/mavlink/{self.proxy_name}/tx\n"
            f"  RX (actuator): /actuator/mavlink/{self.proxy_name}/rx"
        )
        
        # Start TCP server to accept PX4 SITL connections
        self._server = await asyncio.start_server(
            self._handle_px4_connection,
            self.configuration.px4_host,
            self.configuration.px4_port
        )
        addr = self._server.sockets[0].getsockname()
        self.loginfo(f"MAVLink proxy listening on {addr[0]}:{addr[1]}")
        
        # Start background task to read from PX4
        read_task = asyncio.create_task(self._read_from_px4_task(tx_publisher))
        
        # Run forever
        try:
            async with self._server:
                await asyncio.gather(
                    self._server.serve_forever(),
                    self.join()
                )
        finally:
            read_task.cancel()
            try:
                await read_task
            except asyncio.CancelledError:
                pass
    
    def on_shutdown(self):
        """Shutdown procedure."""
        self.loginfo("Shutting down MAVLink proxy")
        self._disconnect()
        
        # Close server
        if self._server:
            self._server.close()
        
        # Cleanup HIL support
        if hasattr(self, '_hil_tx'):
            self._hil_tx.deinit_hil_support()
        if hasattr(self, '_hil_rx'):
            self._hil_rx.deinit_hil_support()


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument(
        "--proxy-name",
        type=str,
        default="mavlink",
        help="Name of the MAVLink proxy instance"
    )
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Name of the configuration"
    )
    args: argparse.Namespace = parser.parse_args()
    
    # create node
    node: MAVLinkProxyNode = MAVLinkProxyNode(
        config=args.config,
        proxy_name=args.proxy_name
    )
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
