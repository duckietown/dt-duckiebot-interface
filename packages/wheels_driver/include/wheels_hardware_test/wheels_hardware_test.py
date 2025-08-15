import time
from typing import Any

from dt_duckiebot_hardware_tests import AbstractHardwareTest
from dtps import DTPSContext
from duckietown_messages.standard.dictionary import Dictionary
from wheels_driver.wheels_driver_abs import WheelsDriverAbs


class WheelsHardwareTest(AbstractHardwareTest):
    _driver: WheelsDriverAbs

    def __init__(self, node: Any, test_out_queue: DTPSContext, driver: WheelsDriverAbs) -> None:
        super().__init__(node, test_out_queue)
        self._driver = driver

    async def run_test(self, dictionary: Dictionary) -> None:
        test_id = dictionary.data["test_id"]
        info_str = dictionary.data["info_str"]
        speed = dictionary.data["speed"]
        duration = dictionary.data["duration"]
        self.message = f"[{test_id}] speed = {speed}, duration = {duration}s"
        self._node.running_test = True
        if info_str == "left":
            self._driver.set_wheels_speed(speed, 0)
        else:
            self._driver.set_wheels_speed(0, speed)
        time.sleep(duration)
        self._driver.set_wheels_speed(0, 0)
        self._node.running_test = False
