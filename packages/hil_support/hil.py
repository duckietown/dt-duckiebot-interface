#!/usr/bin/env python3

import logging
import traceback
from enum import IntEnum
from typing import Optional, List, Dict, Callable

from dtps import context, DTPSContext
from dtps_http import RawData
from dtps_utils.passthrough import DTPSPassthrough
from duckietown_messages.network.dtps.context import DTPSContextMsg
from duckietown_messages.simulation.hil.configuration import HILConfiguration
from duckietown_messages.simulation.hil.connection.configuration import HILConnectionConfiguration
from duckietown_messages.utils.exceptions import DataDecodingError
from kvstore_utils import KVStore

logger = logging.getLogger("hil-support")


class HardwareInTheLoopSide(IntEnum):
    SOURCE = 0
    DESTINATION = 1


class HardwareInTheLoopSupport:
    """
    Handles the support of hardware in the loop (HIL) simulation.
    """

    def __init__(self):
        self._side: Optional[HardwareInTheLoopSide] = None
        # passthrough context
        self._passthrough: Optional[DTPSPassthrough] = None
        self._hil_configuration: HILConfiguration = HILConfiguration()

    @property
    def hil_is_active(self) -> bool:
        return self._passthrough is not None and self._passthrough.is_active

    async def init_hil_support(
            self,
            node: DTPSContext,
            src: Optional[DTPSContext],
            src_path: List[str],
            dst: Optional[DTPSContext],
            dst_path: List[str],
            subpaths: List[str],
            side: HardwareInTheLoopSide,
            transformations: Optional[Dict[str, Callable[[RawData], RawData]]] = None,
            ):
        """
        Configures the support for hardware in the loop (HIL) simulation.
        """
        self._side: HardwareInTheLoopSide = side
        # create passthrough
        self._passthrough = DTPSPassthrough(node, src, dst, subpaths, src_path=src_path, dst_path=dst_path,
                                            transformations=transformations)
        await self._passthrough.astart()
        # kv-store
        kvstore: KVStore = KVStore()
        await kvstore.init()
        # declare the configuration keys
        await kvstore.declare("hil/connection", value=None, persist=False)
        await kvstore.declare("hil/configuration", value=None, persist=False)
        # subscribe to updates to the HIL configuration
        await kvstore.subscribe("hil/connection", self._on_hil_connection_cfg_update)
        await kvstore.subscribe("hil/configuration", self._on_hil_cfg_update)

    async def _on_hil_connection_cfg_update(self, rd: RawData):
        """
        Callback for HIL connection configuration updates.
        """
        try:
            conn_cfg: Optional[HILConnectionConfiguration] = (
                HILConnectionConfiguration.from_rawdata(rd, allow_none=True))
        except DataDecodingError as e:
            logger.error(f"Failed to decode the HIL connection configuration. Ignoring.")
            logger.debug(f"Error is:\n{e.message}")
            return
        remote: Optional[DTPSContext] = None
        path: List[str] = []
        if conn_cfg is not None:
            cxt_cfg: Optional[DTPSContextMsg] = conn_cfg.simulator
            if cxt_cfg is None:
                logger.info(f"Field 'simulator' in connection configuration is 'None'. This is not expected. Ignoring.")
                return
            # open link to context indicated
            try:
                remote: DTPSContext = await context(cxt_cfg.name, urls=cxt_cfg.urls)
            except Exception as e:
                logger.error(f"Failed to open context '{cxt_cfg.name}':"
                             f"\n\turls: {cxt_cfg.urls}"
                             f"\n\terror: {str(e)}")
                return
            # make sure the context exists
            exists: bool = False
            try:
                exists = await remote.exists()
            except TimeoutError:
                pass
            if not exists:
                logger.info(f"Remote context '{cxt_cfg.name}' with urls '{cxt_cfg.urls}' does not exist or "
                            f"it is not reachable. Ignoring.")
                return
            # navigate to optional path
            if cxt_cfg.path:
                remote = remote.navigate(cxt_cfg.path)
            # move path to the agent level
            path.append(conn_cfg.agent_name)
            # connect passthrough
            logger.info(f"Connecting passthrough to context '{cxt_cfg.name}' with configuration {cxt_cfg} and "
                        f"path {'/'.join(path)}")
        else:
            if self._passthrough.is_active:
                logger.info(f"Disconnecting passthrough")

        if self._side == HardwareInTheLoopSide.SOURCE:
            # set source
            try:
                await self._passthrough.set_source(remote, path)
            except Exception as e:
                logger.error(f"Failed to set passthrough source: {str(e)}")
                traceback.print_exc()
        elif self._side == HardwareInTheLoopSide.DESTINATION:
            # set destination
            try:
                await self._passthrough.set_destination(remote, path)
            except Exception as e:
                logger.error(f"Failed to set passthrough destination: {str(e)}")
                traceback.print_exc()

    async def _on_hil_cfg_update(self, rd: RawData):
        """
        Callback for HIL configuration updates.
        """
        try:
            cfg: Optional[HILConfiguration] = HILConfiguration.from_rawdata(rd, allow_none=True)
        except DataDecodingError as e:
            logger.error(f"Failed to decode HIL configuration: {e.message}")
            return
        # default configuration
        if cfg is None:
            cfg = HILConfiguration()
        # set the configuration
        self._hil_configuration = cfg

    def deinit_hil_support(self):
        if self._passthrough is not None:
            self._passthrough.stop()
        logger.info("Passthrough stopped.")
