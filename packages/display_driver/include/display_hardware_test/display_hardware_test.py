import time
from typing import Any

from dt_duckiebot_hardware_tests import AbstractHardwareTest
from dtps import DTPSContext
from duckietown_messages.actuators.display_fragment import DisplayFragment
from duckietown_messages.sensors.image import Image
from duckietown_messages.standard.dictionary import Dictionary
from duckietown_messages.geometry_2d.roi import ROI

from display_driver.luma.ssd1306 import SSD1306Display
from display_driver.types.page import PAGE_HOME, PAGE_TEST_DISPLAY
from display_driver.types.regions import DisplayRegionID
from display_renderer import monospace_screen


class DisplayHardwareTest(AbstractHardwareTest):
    _display: SSD1306Display

    def __init__(self, node: Any, test_out_queue: DTPSContext, display: SSD1306Display) -> None:
        super().__init__(node, test_out_queue)
        self._display = display

    @staticmethod
    def _get_fragment(text: str) -> DisplayFragment:
        im = monospace_screen((32, 128), text, scale="hfill")
        content = Image.from_np(im, encoding="mono8")
        location = ROI(x=0, y=8, width=128, height=32)
        return DisplayFragment(
            name="__display_test__",
            region=DisplayRegionID.BODY,
            page=PAGE_TEST_DISPLAY,
            content=content,
            location=location,
            z=0,
            ttl=-1
        )

    async def run_test(self, dictionary: Dictionary) -> None:
        test_id = dictionary.data["test_id"]
        duration = dictionary.data["duration"]
        text = dictionary.data["text"]
        self.message = f"[{test_id}] duration = {duration}, text = '{text}'"
        fragment = self._get_fragment(text)
        self._node.running_test = True
        self._display.add_fragment(fragment)
        self._display.page = PAGE_TEST_DISPLAY
        time.sleep(duration)
        with self._display.fragments_lock:
            del self._display.fragments[DisplayRegionID.BODY][fragment.name]
        self._display.page = PAGE_HOME
        self._node.running_test = False
