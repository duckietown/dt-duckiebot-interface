"""Announce live Matrix HIL presence from one node per robot."""

import asyncio
import logging
import socket
from ipaddress import ip_address
from typing import Optional, Sequence
from urllib.parse import urlsplit
from uuid import uuid4

from aiohttp import ClientError
from dtps import DTPSContext, context
from dtps_http import RawData
from dt_robot_utils import RobotHardware, get_robot_hardware
from duckietown_messages.simulation.hil.connection.configuration import HILConnectionConfiguration
from hil_support.hil import get_hil_robot_model
from kvstore_utils import KVStore


logger = logging.getLogger("hil-support")
HEARTBEAT_SECONDS = 2.0
REQUEST_TIMEOUT = 1.5
try:
    from dtps.ergo_use import CannotConnectToAnyURL
    CONNECTION_ERRORS = (OSError, asyncio.TimeoutError, ClientError, CannotConnectToAnyURL)
except ImportError:
    CONNECTION_ERRORS = (OSError, asyncio.TimeoutError, ClientError)


def _route_address(urls: Sequence[str]) -> Optional[str]:
    """Get the robot's local address on the route to the Engine, without sending traffic."""
    for url in urls:
        try:
            endpoint = urlsplit(url)
            if endpoint.scheme not in ("http", "https") or not endpoint.hostname:
                continue
            port = endpoint.port or (443 if endpoint.scheme == "https" else 80)
            for family, kind, protocol, _, peer in socket.getaddrinfo(
                endpoint.hostname, port, type=socket.SOCK_DGRAM,
            ):
                try:
                    with socket.socket(family, kind, protocol) as route:
                        route.connect(peer)
                        address = ip_address(route.getsockname()[0])
                    if not (address.is_unspecified or address.is_loopback or address.is_multicast):
                        return str(address)
                except OSError:
                    continue
        except (OSError, ValueError):
            continue
    return None


class AttachmentPresence:
    def __init__(self, robot: str):
        self._robot = robot
        self._model = get_hil_robot_model()
        hardware = get_robot_hardware()
        if hardware == RobotHardware.UNKNOWN:
            self._hardware = "unknown"
        else:
            self._hardware = "virtual" if hardware == RobotHardware.VIRTUAL else "physical"
        self._config: Optional[HILConnectionConfiguration] = None
        self._remote: Optional[DTPSContext] = None
        self._announced = False
        self._instance = uuid4().hex

    async def publish_identity(self) -> None:
        # The CLI reads the identity from the same robot KV store it will configure.
        kvstore = KVStore()
        await kvstore.init()
        identity = {"robot": self._robot, **self._model}
        await kvstore.declare("hil/identity", value=identity, persist=False)
        await kvstore.set("hil/identity", identity)

    async def update(self, config: Optional[HILConnectionConfiguration], *, dreamwalking: bool) -> None:
        if config is not self._config:
            await self.close()
            self._config = config
            self._instance = uuid4().hex
        if config is None or config.simulator is None or not config.simulator.urls:
            return
        try:
            if self._remote is None:
                self._remote = await asyncio.wait_for(
                    context(config.simulator.name, urls=config.simulator.urls), REQUEST_TIMEOUT,
                )
            address = None
            try:
                address = await asyncio.wait_for(
                    asyncio.get_running_loop().run_in_executor(None, _route_address, config.simulator.urls),
                    REQUEST_TIMEOUT,
                )
            except asyncio.TimeoutError:
                logger.debug("Could not determine the robot address for HIL presence.")
            # Acknowledgement from the Engine verifies both reachability and the entity key.
            await self._report(attached=True, dreamwalking=dreamwalking, address=address)
        except CONNECTION_ERRORS as error:
            if self._announced:
                logger.info("Lost Duckiematrix HIL presence for %s: %s", config.agent_name, type(error).__name__)
            else:
                logger.debug("Duckiematrix presence unavailable: %s", type(error).__name__)
            self._announced = False
        else:
            self._announced = True

    async def _report(self, *, attached: bool, dreamwalking: bool = False, address: Optional[str] = None) -> None:
        if self._config is None or self._remote is None:
            return
        topic = self._remote / "hil" / "attachments"
        payload = {
            "entity": self._config.agent_name,
            "robot": self._robot,
            "instance": self._instance,
            "attached": attached,
        }
        if attached:
            payload.update(self._model, hardware=self._hardware)
            if self._hardware == "physical":
                payload["dreamwalking"] = dreamwalking
            if address is not None:
                payload["address"] = address
        await asyncio.wait_for(topic.publish(RawData.json_from_native_object(payload)), REQUEST_TIMEOUT)

    async def close(self) -> None:
        remote = self._remote
        if remote is None:
            return
        try:
            if self._announced:
                await self._report(attached=False)
        except CONNECTION_ERRORS as error:
            logger.debug("Letting the Duckiematrix presence lease expire: %s", type(error).__name__)
        finally:
            self._announced = False
            self._remote = None
            await remote.aclose()
