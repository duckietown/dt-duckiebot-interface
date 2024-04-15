#!/usr/bin/env python3

import asyncio
import dataclasses
import os
from pathlib import Path
from typing import Union, Optional, Dict, Awaitable, Callable, List

import argparse
import netifaces
import requests
from PIL import Image, ImageOps

from display_driver.types import Z_SYSTEM
from display_driver.types.page import ALL_PAGES, PAGE_HOME, PAGE_ROBOT_INFO
from display_driver.types.regions import REGION_HEADER, REGION_BODY
from display_driver.types.roi import DisplayROI
from display_renderer import AbsDisplayFragmentRenderer, TextFragmentRenderer
from display_renderer.renderer import MultipageTextFragmentRenderer
from display_renderer.text import monospace_screen
from dt_node_utils import NodeType
from dt_node_utils.asyncio import create_task
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.decorators import sidecar
from dt_node_utils.node import Node
from dt_robot_utils import get_robot_name, get_robot_configuration
from dtps.ergo_ui import PublisherInterface
from duckietown_messages.actuators.display_fragments import DisplayFragments
from duckietown_messages.utils.image.pil import pil_to_np


@dataclasses.dataclass
class DisplayRendererNodeConfiguration(NodeConfiguration):

    @dataclasses.dataclass
    class RendererConfiguration:
        frequency: float

    renderers: Dict[str, RendererConfiguration]


class DisplayRendererNode(Node):

    def __init__(self, config: str):
        node_name: str = "display-renderer"
        super().__init__(
            name=node_name,
            kind=NodeType.VISUALIZATION,
            description="Display renderers node",
        )
        # configuration
        self.configuration = DisplayRendererNodeConfiguration.from_name(self.package, node_name, config)
        # assets dir
        assets_dir: Path = self._package.assets_dir
        # health data
        self._health_data: Optional[dict] = None
        # queues
        self._fragments: Optional[PublisherInterface] = None
        # health API url (use mDNS only if we are not deployed onboard)
        self._health_api_url: str = "http://localhost/health/" if os.environ.get("DT_DEPLOYED_ONBOARD", "0") == "1" \
            else f"http://{self._robot_name}.local/health/"
        # create renderers
        self._battery_indicator = BatteryIndicatorFragmentRenderer(
            assets_dir,
            self.publish,
            frequency=self.configuration.renderers["health"].frequency
        )
        self._usage_renderer = UsageStatsFragmentRenderer(
            self.publish,
            frequency=self.configuration.renderers["health"].frequency
        )
        self._robot_info_renderer = RobotInfoRenderer(
            self.publish,
            frequency=self.configuration.renderers["robot_info"].frequency
        )
        self._wlan0_indicator = NetIFaceFragmentRenderer(
            assets_dir,
            "wlan0",
            DisplayROI(0, 0, 11, 16),
            self.publish,
            frequency=self.configuration.renderers["network"].frequency
        )
        self._eth0_indicator = NetIFaceFragmentRenderer(
            assets_dir,
            "eth0",
            DisplayROI(14, 0, 11, 16),
            self.publish,
            frequency=self.configuration.renderers["network"].frequency
        )
        self._renderers: Dict[str, AbsDisplayFragmentRenderer] = {
            "battery_indicator": self._battery_indicator,
            "usage_renderer": self._usage_renderer,
            "robot_info_renderer": self._robot_info_renderer,
            "wlan0_indicator": self._wlan0_indicator,
            "eth0_indicator": self._eth0_indicator
        }

    async def worker(self):
        await self.dtps_init(self.configuration)
        # create fragments queue
        queue = await (self.switchboard / "actuator" / "display" / "fragments").until_ready()
        self._fragments = await queue.publisher()
        # expose node to the switchboard
        await self.dtps_expose()
        # run forever
        await asyncio.wait([
            create_task(renderer.worker, name, self.logger) for name, renderer in self._renderers.items()
        ])

    @sidecar
    async def health_data_fetcher(self):
        dt: float = 1.0 / self.configuration.renderers["health"].frequency
        while True:
            # noinspection PyBroadException
            try:
                # TODO: the health API should expose a DTPS queue with customizable frequency
                health_data = requests.get(self._health_api_url).json()
            except BaseException:
                await asyncio.sleep(dt)
                continue
            # update health renderers
            for renderer in [self._battery_indicator, self._usage_renderer, self._robot_info_renderer]:
                renderer.update(health_data)
            # ---
            await asyncio.sleep(dt)

    async def publish(self, renderer: AbsDisplayFragmentRenderer):
        await self._fragments.publish(DisplayFragments(
            fragments=renderer.fragments
        ).to_rawdata())


class BatteryIndicatorFragmentRenderer(AbsDisplayFragmentRenderer):

    def __init__(self,
                 assets_dir: Path,
                 callback: Callable[['AbsDisplayFragmentRenderer'], Awaitable],
                 frequency: float):
        super(BatteryIndicatorFragmentRenderer, self).__init__(
            "__battery_indicator__",
            page=ALL_PAGES,
            region=REGION_HEADER,
            roi=DisplayROI(90, 0, 38, 16),
            z=Z_SYSTEM,
            callback=callback,
            frequency=frequency
        )
        self._assets_dir = assets_dir
        self._percentage: int = -1
        self._charging: bool = False
        self._present: bool = False
        # load assets
        self._assets = {
            asset: pil_to_np(ImageOps.grayscale(Image.open(self._assets_dir / "icons" / f"{asset}.png")))
            for asset in [
                "battery_not_found",
                "battery_charging",
                "battery_0",
                "battery_1",
                "battery_2",
                "battery_3",
                "battery_4",
                "battery_5",
                "battery_6",
                "battery_7",
                "battery_8",
                "battery_9",
                "battery_10",
            ]
        }

    def _draw_indicator(self, icon: str, text: str):
        indicator_icon = self._assets[icon]
        ico_h, ico_w = indicator_icon.shape
        ico_space = 2
        # draw icon
        self._buffer[0:ico_h, 0:ico_w] = indicator_icon
        # draw text
        vshift_px = 2
        text_h, text_w = 14, self._roi.w - ico_w - ico_space
        text_buf = monospace_screen((text_h, text_w), text, scale="fill")
        self._buffer[vshift_px:vshift_px + text_h, ico_w + ico_space:] = text_buf

    def update(self, data: dict):
        self._present: bool = data["battery"]["present"]
        self._charging: bool = data["battery"]["charging"]
        self._percentage: int = data["battery"]["percentage"]

    def render(self):
        # no data yet
        if self._percentage < 0:
            return
        # battery not found
        if not self._present:
            self._draw_indicator("battery_not_found", "NoBT")
            return
        # battery charging/discharging
        if self._charging:
            # battery charging
            _icon = "battery_charging"
        else:
            # battery discharging
            dec = "%d" % (self._percentage / 10)
            _icon = f"battery_{dec}"
        # draw icon
        self._draw_indicator(_icon, "%d%%" % self._percentage)


class UsageStatsFragmentRenderer(TextFragmentRenderer):

    BAR_LEN = 11
    BAR_SYMBOL = "#"
    CANVAS = """\
TMP |{ctmp_bar}| {ctmp}
CPU |{pcpu_bar}| {pcpu}
RAM |{pmem_bar}| {pmem}
DSK |{pdsk_bar}| {pdsk}"""

    def __init__(self,
                 callback: Callable[['AbsDisplayFragmentRenderer'], Awaitable],
                 frequency: float):
        super(UsageStatsFragmentRenderer, self).__init__(
            "__usage_stats__",
            page=PAGE_HOME,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
            scale="fill",
            callback=callback,
            frequency=frequency
        )
        self._min_ctmp = 20
        self._max_ctmp = 80
        # data
        self._ctmp: Union[None, str, int, float] = None
        self._pcpu: Union[None, str, int] = None
        self._pmem: Union[None, str, int] = None
        self._pdsk: Union[None, str, int] = None

    def update(self, data: dict):
        self._ctmp = data["temperature"]
        self._pcpu = data["cpu"]["percentage"]
        self._pmem = data["memory"]["percentage"]
        self._pdsk = data["disk"]["percentage"]
        # ---
        ptmp = (
            int(100 * (max(0, self._ctmp - self._min_ctmp) / (self._max_ctmp - self._min_ctmp)))
            if isinstance(self._ctmp, (int, float))
            else 0
        )
        text = self.CANVAS.format(
            **{
                "ctmp": self._fmt(self._ctmp, "C"),
                "pcpu": self._fmt(min(self._pcpu, 99), "%"),
                "pmem": self._fmt(min(self._pmem, 99), "%"),
                "pdsk": self._fmt(min(self._pdsk, 99), "%"),
                "ctmp_bar": self._bar(ptmp),
                "pcpu_bar": self._bar(self._pcpu),
                "pmem_bar": self._bar(self._pmem),
                "pdsk_bar": self._bar(self._pdsk),
            }
        )
        super().update(text)

    def render(self):
        # no data yet
        if self._ctmp is None:
            return
        # render
        super().render()

    @staticmethod
    def _fmt(value: Union[str, int], suffix: str):
        if isinstance(value, str):
            return f"ERR"
        return f"{int(value)}{suffix}"

    @classmethod
    def _bar(cls, value: Union[str, int], scale: int = 100):
        if isinstance(value, str):
            return f"ERR"
        value /= scale
        full = int(cls.BAR_LEN * value)
        return cls.BAR_SYMBOL * full + " " * (cls.BAR_LEN - full)


class NetIFaceFragmentRenderer(AbsDisplayFragmentRenderer):

    def __init__(self,
                 assets_dir: Path,
                 iface: str,
                 roi: DisplayROI,
                 callback: Callable[['AbsDisplayFragmentRenderer'], Awaitable],
                 frequency: float):
        super(NetIFaceFragmentRenderer, self).__init__(
            f"__iface_connection_{iface}__",
            page=ALL_PAGES,
            region=REGION_HEADER,
            roi=roi,
            z=Z_SYSTEM,
            ttl=30,
            callback=callback,
            frequency=frequency
        )
        self._assets_dir = assets_dir
        self._iface = iface
        # load assets
        self._assets = {
            asset: pil_to_np(ImageOps.grayscale(Image.open(self._assets_dir / "icons" / f"{asset}.png")))
            for asset in [f"{self._iface}_connected", f"{self._iface}_not_connected"]
        }

    def render(self):
        # fetch info about iface
        try:
            iface_addrs = netifaces.ifaddresses(self._iface)
            connected = netifaces.AF_INET in iface_addrs
        except ValueError:
            connected = False
        icon = self._iface + ("" if connected else "_not") + "_connected"
        self._buffer[:, :] = self._assets[icon]


class RobotInfoRenderer(MultipageTextFragmentRenderer):

    def __init__(
            self,
            callback: Callable[['AbsDisplayFragmentRenderer'], Awaitable],
            frequency: float
    ):
        super(RobotInfoRenderer, self).__init__(
            name="__robot_info__",
            page=PAGE_ROBOT_INFO,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
            scale="hfill",
            lines_per_page=3,
            ttl=-1,
            callback=callback,
            frequency=frequency
        )
        # data
        self._data: Optional[dict] = None

    @staticmethod
    def _shorten_str(value):
        """If the input is longer than the allowed, it's trimmed and '...' is added"""
        # maximum length of the value
        max_length: int = 8
        output = value
        if len(value) > max_length:
            output = value[:(max_length - 3)] + "..."
        return output

    def _fmt(self, data: Dict[str, str]) -> str:
        # length of the longest line
        maxl: int = max([
            len(k) + len(self._shorten_str(v)) + 1 for k, v in data.items()
        ])
        # compile text
        text: str = ""
        for k, v in data.items():
            if len(k) > 0:
                w = self._shorten_str(v)
                text += f"{k}{' ' * (maxl - len(k) - len(w))}{w}\n"
            else:
                text += f"{v}\n"
        return text.strip("\n")

    def update(self, data: dict):
        self._data = data

    async def step(self):
        if self._data is None:
            return
        firmware: str = f"v{self._data['software']['version']}"
        distro: str = os.environ.get('DT_DISTRO', 'N.A.')
        ip: str = self.get_local_ip_address_on_gateway_interface() or "N.A."
        # format texts for the display
        text: str = self._fmt({
            "Name": get_robot_name(),
            "Model": get_robot_configuration().name,
            "Firmware": firmware,
            "Distro": distro,
            # NOTE: IP uses two lines
            "IP": "", "": ip
        })
        # update the underlying text renderer
        super().update(text)

    @staticmethod
    def get_local_ip_address_on_gateway_interface() -> Optional[str]:
        gateway_iface = netifaces.gateways()['default'][netifaces.AF_INET][1]
        ips: List[dict] = netifaces.ifaddresses(gateway_iface).get(netifaces.AF_INET)
        if ips:
            return ips[0]['addr']
        return None


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True, help="Name of the configuration")
    args: argparse.Namespace = parser.parse_args()
    # create node
    node: DisplayRendererNode = DisplayRendererNode(config=args.config)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
