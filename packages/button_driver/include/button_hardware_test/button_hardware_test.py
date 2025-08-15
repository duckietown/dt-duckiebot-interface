import time
from typing import Any

from dt_duckiebot_hardware_tests import AbstractHardwareTest
from dtps import DTPSContext
from duckietown_messages.standard.dictionary import Dictionary

from button_driver import ButtonDriver


class ButtonHardwareTest(AbstractHardwareTest):
    _button_released: bool
    _driver: ButtonDriver

    def __init__(self, node: Any, test_out_queue: DTPSContext, driver: ButtonDriver) -> None:
        super().__init__(node, test_out_queue)
        self._driver = driver
        self._button_released = False

    def _button_event_cb(self) -> None:
        self._button_released = True

    async def run_test(self, dictionary: Dictionary) -> None:
        test_id = dictionary.data["test_id"]
        test_timeout = dictionary.data["test_timeout"]
        led_blink_secs = dictionary.data["led_blink_secs"]
        led_blink_hz = dictionary.data["led_blink_hz"]
        self.message = f"[{test_id}] led_blink_secs = {led_blink_secs}, led_blink_hz = {led_blink_hz}"
        # button led test
        self._driver.led.blink(led_blink_secs, led_blink_hz)
        # button press event test
        self._driver.start_test(self._button_event_cb)
        sleep_duration = 0.1
        counter = 0
        while not self._button_released:
            if counter > test_timeout:
                message = f"[{test_id}] Button not released in time."
                raise Exception(message)
            if round(counter, 1) % 1 == 0:
                self._node.loginfo(f"[{test_id}] Waiting for button to be released... ({round(counter)}/{test_timeout} seconds)")
            time.sleep(sleep_duration)
            counter += sleep_duration
        self._node.loginfo(f"[{test_id}] Button released.")
        # reset
        self._button_released = False
