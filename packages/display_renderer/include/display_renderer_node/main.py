#!/usr/bin/env python3

import asyncio
import dataclasses
import time
from asyncio import Task
from pathlib import Path
from threading import Thread
from typing import Union, Optional, Dict, Awaitable, Callable

import argparse
import netifaces
import requests
from PIL import Image, ImageOps
from dtps.ergo_ui import PublisherInterface

from display_driver.types import Z_SYSTEM
from display_driver.types.page import ALL_PAGES, PAGE_HOME
from display_driver.types.regions import REGION_HEADER, REGION_BODY
from display_driver.types.roi import DisplayROI
from display_renderer import AbsDisplayFragmentRenderer, TextFragmentRenderer
from display_renderer.text import monospace_screen
from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.node import Node
from duckietown_messages.actuators.display_fragments import DisplayFragments
from duckietown_messages.standard.header import Header
from duckietown_messages.utils.image.pil import pil_to_np


@dataclasses.dataclass
class DisplayRendererNodeConfiguration(NodeConfiguration):

    @dataclasses.dataclass
    class RendererConfiguration:
        frequency: float

    renderers: Dict[str, RendererConfiguration]


class DisplayRendererNode(Node):

    def __init__(self, config: str):
        super().__init__(
            name="display-renderer",
            kind=NodeType.VISUALIZATION,
            description="Display renderers node",
        )
        # configuration
        self._configuration = DisplayRendererNodeConfiguration.from_name(self.package, config)
        # assets dir
        assets_dir: Path = self._package.assets_dir
        # queues
        self._fragments: Optional[PublisherInterface] = None
        # data fetcher worker
        self._fetcher: HealthDataFetcher = HealthDataFetcher(
            url=f"http://{self._robot_name}.local/health/",
            frequency=self._configuration.renderers["health"].frequency
        )
        self._fetcher.start()
        # create renderers
        self._battery_indicator = BatteryIndicatorFragmentRenderer(
            assets_dir,
            self._fetcher,
            self.publish,
            frequency=self._configuration.renderers["health"].frequency
        )
        self._usage_renderer = UsageStatsFragmentRenderer(
            self._fetcher,
            self.publish,
            frequency=self._configuration.renderers["health"].frequency
        )
        self._wlan0_indicator = NetIFaceFragmentRenderer(
            assets_dir,
            "wlan0",
            DisplayROI(0, 0, 11, 16),
            self.publish,
            frequency=self._configuration.renderers["network"].frequency
        )
        self._eth0_indicator = NetIFaceFragmentRenderer(
            assets_dir,
            "eth0",
            DisplayROI(14, 0, 11, 16),
            self.publish,
            frequency=self._configuration.renderers["network"].frequency
        )
        self._renderers = [self._battery_indicator, self._usage_renderer, self._wlan0_indicator, self._eth0_indicator]

    async def worker(self):
        await self.dtps_init(self._configuration)
        # create fragments queue
        queue = await (self.switchboard / "actuator" / "display" / "fragments").until_ready()
        self._fragments = await queue.publisher()
        # expose node to the switchboard
        await self.dtps_expose()
        # run forever
        await asyncio.wait([
            Task(renderer.worker()) for renderer in self._renderers
        ])

    async def publish(self, renderer: AbsDisplayFragmentRenderer):
        await self._fragments.publish(DisplayFragments(
            header=Header(),
            fragments=renderer.fragments
        ).to_rawdata())


class HealthDataFetcher(Thread):

    def __init__(self, url: str, frequency: float):
        super().__init__(daemon=True, name="health-data-fetcher")
        self._url: str = url
        self._frequency: float = frequency
        self.data: Optional[dict] = None

    def run(self):
        dt: float = 1.0 / self._frequency
        while True:
            # noinspection PyBroadException
            try:
                self.data = requests.get(self._url).json()
            except BaseException:
                pass
            time.sleep(dt)


class BatteryIndicatorFragmentRenderer(AbsDisplayFragmentRenderer):

    def __init__(self,
                 assets_dir: Path,
                 fetcher: HealthDataFetcher,
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
        self._fetcher = fetcher
        self._assets_dir = assets_dir
        self._percentage = 0
        self._charging = False
        self._present = False
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

    def render(self):
        if self._fetcher.data is None:
            return
        # get data
        data: dict = self._fetcher.data["battery"]
        present: bool = data["present"]
        charging: bool = data["charging"]
        percentage: int = data["percentage"]
        # battery not found
        if not present:
            self._draw_indicator("battery_not_found", "NoBT")
            return
        # battery charging
        if charging:
            _icon = "battery_charging"
        else:
            dec = "%d" % (percentage / 10)
            _icon = f"battery_{dec}"
        # battery discharging
        self._draw_indicator(_icon, "%d%%" % percentage)


class UsageStatsFragmentRenderer(TextFragmentRenderer):

    BAR_LEN = 11
    BAR_SYMBOL = "#"
    CANVAS = """\
TMP |{ctmp_bar}| {ctmp}
CPU |{pcpu_bar}| {pcpu}
RAM |{pmem_bar}| {pmem}
DSK |{pdsk_bar}| {pdsk}"""

    def __init__(self,
                 fetcher: HealthDataFetcher,
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
        self._fetcher = fetcher
        self._min_ctmp = 20
        self._max_ctmp = 60

    def render(self):
        if self._fetcher.data is None:
            return
        # get data
        data: dict = self._fetcher.data
        ctmp: Union[str, int, float] = data["temperature"]
        pcpu: Union[str, int] = data["cpu"]["percentage"]
        pmem: Union[str, int] = data["memory"]["percentage"]
        pdsk: Union[str, int] = data["disk"]["percentage"]
        # ---
        ptmp = (
            int(100 * (max(0, ctmp - self._min_ctmp) / (self._max_ctmp - self._min_ctmp)))
            if isinstance(ctmp, (int, float))
            else 0
        )
        text = self.CANVAS.format(
            **{
                "ctmp": self._fmt(ctmp, "C"),
                "pcpu": self._fmt(pcpu, "%"),
                "pmem": self._fmt(pmem, "%"),
                "pdsk": self._fmt(pdsk, "%"),
                "ctmp_bar": self._bar(ptmp),
                "pcpu_bar": self._bar(pcpu),
                "pmem_bar": self._bar(pmem),
                "pdsk_bar": self._bar(pdsk),
            }
        )
        self.update(text)
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
