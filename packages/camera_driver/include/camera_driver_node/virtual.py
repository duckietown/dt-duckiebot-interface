#!/usr/bin/env python3

import argparse
import asyncio
import traceback
from typing import Optional

from camera_driver import CameraNodeAbs
from dtps import context, DTPSContext
from dtps_http import RawData
from dtps_utils.passthrough import DTPSPassthrough
from duckietown_messages.network.dtps.context import DTPSContextMsg
from duckietown_messages.simulation.hil.configuration import HILConfiguration
from duckietown_messages.simulation.hil.connection.configuration import HILConnectionConfiguration
from duckietown_messages.utils.exceptions import DataDecodingError
from kvstore.client.client import KVStore

import coloredlogs
coloredlogs.install()


class CameraNode(CameraNodeAbs):
    """
    Handles the imagery on a Virtual robot.
    """

    def __init__(self, config: str, sensor_name: str):
        # Initialize the DTROS parent class
        super(CameraNode, self).__init__(config, sensor_name)
        # passthrough context
        self._passthrough: Optional[DTPSPassthrough] = None
        self._hil_configuration: Optional[HILConfiguration] = None
        # ---
        self.loginfo("[CameraNode]: Initialized.")

    async def _on_hil_connection_cfg_update(self, rd: RawData):
        """
        Callback for HIL connection configuration updates.
        """
        try:
            conn_cfg: Optional[HILConnectionConfiguration] = (
                HILConnectionConfiguration.from_rawdata(rd, allow_none=True))
        except DataDecodingError as e:
            self.logerr(f"Failed to decode the HIL connection configuration. Ignoring.")
            self.logdebug(f"Error is:\n{e.message}")
            return
        source: Optional[DTPSContext] = None
        if conn_cfg is not None:
            cxt_cfg: Optional[DTPSContextMsg] = conn_cfg.source
            if cxt_cfg is not None:
                # open link to context indicated
                try:
                    source: DTPSContext = await context(cxt_cfg.name, urls=cxt_cfg.urls)
                except Exception as e:
                    self.logerr(f"Failed to open context '{cxt_cfg.name}':"
                                f"\n\turls: {cxt_cfg.urls}"
                                f"\n\terror: {str(e)}")
                    return
                # navigate to optional path
                if cxt_cfg.path:
                    source = source.navigate(cxt_cfg.path)
                # navigate to the agent level
                source = source.navigate(conn_cfg.agent_name)
                # navigate to the camera topic
                source = source.navigate("sensor", "camera", self.sensor_name)
                # connect passthrough
                self.loginfo(f"Connecting passthrough to context '{cxt_cfg.name}' with configuration {cxt_cfg} and "
                             f"path '{'/'.join(source.get_path_components())}'.")
                # make sure the context exists
                exists: bool = await source.exists()
                if not exists:
                    self.loginfo(f"Source context '{cxt_cfg.name}' does not exist or it is not reachable. Ignoring")
                    return
            else:
                self.logerr(f"Source context is None. This will trigger a disconnection")
        else:
            if self._passthrough.is_active:
                self.loginfo(f"Disconnecting passthrough")
        try:
            await self._passthrough.set_source(source)
        except Exception as e:
            self.logerr(f"Failed to set passthrough source: {str(e)}")
            traceback.print_exc()

    async def _on_hil_cfg_update(self, rd: RawData):
        """
        Callback for HIL configuration updates.
        """
        try:
            cfg: Optional[HILConfiguration] = HILConfiguration.from_rawdata(rd, allow_none=True)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode HIL configuration: {e.message}")
            return
        # default configuration
        if cfg is None:
            cfg = HILConfiguration()
        # set the configuration
        self._hil_configuration = cfg

    async def worker(self):
        """
        Image capture procedure.

        Captures frames from the camera and publishes them.
        """
        # init queues
        await self.dtps_init_queues()
        # kv-store
        kvstore: KVStore = KVStore()
        await kvstore.init()
        # subscribe to updates to the HIL configuration
        await kvstore.on_update(
            "hil/connection/configuration",
            self._on_hil_connection_cfg_update,
            create_if_missing=True,
            initial_value=None
        )
        await kvstore.on_update(
            "hil/configuration",
            self._on_hil_cfg_update,
            create_if_missing=True,
            initial_value=None
        )
        # keep loop alive
        while not self.is_shutdown:
            await asyncio.sleep(1.0)
        self.loginfo("Camera worker stopped.")

    def setup(self):
        pass

    def on_shutdown(self):
        if self._passthrough is not None:
            self._passthrough.stop()
        self.loginfo("Passthrough stopped.")


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
