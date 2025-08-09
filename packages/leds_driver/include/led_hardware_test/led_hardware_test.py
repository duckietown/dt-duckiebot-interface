import numpy as np
import time
from typing import Any, Dict, List, Tuple, Optional

from colorir import HSV
from dtps import DTPSContext
from duckietown_messages.standard.dictionary import Dictionary

from dt_duckiebot_hardware_tests import AbstractHardwareTest
from leds_driver import LEDsDriverAbs


class LEDHardwareTest(AbstractHardwareTest):
    _color_sequence: Optional[List[Tuple[float, float, float]]]
    _driver: LEDsDriverAbs
    _idle_lighting: Dict[str, List]
    _led_tuple: tuple[str, str, str, str]

    def __init__(
        self,
        node: Any,
        test_out_queue: DTPSContext,
        driver: LEDsDriverAbs,
        led_tuple: tuple[str, str, str, str],
        idle_lighting: Dict[str, List],
    ) -> None:
        super().__init__(node, test_out_queue)
        self._driver = driver
        self._led_tuple = led_tuple
        self._idle_lighting = idle_lighting  # set to this after the test
        self._color_sequence = None  # lazy init. If test is run, generate this

    def _fade_mono(self, led_ids, fade_duration: float, interval: float, fade_in: bool = True, mono_hue: int = 0) -> None:
        """fade IN or OUT in a mono color"""
        # number of different colors to show
        number_of_iterations = int(fade_duration / interval)
        step_size = float(1 / number_of_iterations)
        # increasing brightness
        seq_v: Any = np.arange(0, 1, step_size)
        if not fade_in:
            # or decreasing
            seq_v = reversed(seq_v)
        for v in seq_v:
            for led_id in led_ids:
                hsv = HSV(mono_hue, 1, v)
                color = hsv.rgb()
                self._driver.set_rgb(led_id, color)
            time.sleep(interval)

    def _generate_colors(self, step_size: int = 1) -> List[Tuple[float, float, float]]:
        """Generate a smooth transition of colors"""
        colors = []
        for hue in range(0, 360, step_size):
            hsv = HSV(hue, 1, 1)
            color = hsv.rgb()
            colors.append(color)
        return colors

    async def run_test(self, dictionary: Dictionary) -> None:
        test_id = dictionary.data["test_id"]
        led_ids = dictionary.data["led_ids"]
        fade_in_duration = dictionary.data["fade_in_duration"]
        duration = dictionary.data["duration"]
        fade_out_duration = dictionary.data["fade_out_duration"]
        self.message = f"[{test_id}] fade_in_duration = {fade_in_duration}s, duration = {duration}s, fade_out_duration = {fade_out_duration}s"
        # generate test color sequence if not yet initialized
        if self._color_sequence is None:
            self._color_sequence = self._generate_colors()
        color_sequence_length = len(self._color_sequence)
        interval = duration / float(color_sequence_length)
        self._node.running_test = True
        # turn all on gradually
        self._fade_mono(led_ids, fade_in_duration, interval)
        # run color sequence test
        for color in self._color_sequence:
            for led_id in led_ids:
                self._driver.set_rgb(led_id, color)
            time.sleep(interval)
        # turn all off
        self._fade_mono(led_ids, fade_out_duration, interval, fade_in=False)
        # make sure they are set to idle lighting
        for led_id in led_ids:
            self._driver.set_rgb(led_id, self._idle_lighting[self._led_tuple[led_id]])
        self._node.running_test = False
