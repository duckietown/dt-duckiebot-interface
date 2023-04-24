#!/usr/bin/env python3
import time

import rospy

from rgb_led import RGB_LED
from std_msgs.msg import String, ColorRGBA
from duckietown_msgs.msg import LEDPattern
from duckietown.dtros import DTROS, TopicType, NodeType

from abc import ABC, abstractmethod
from typing import Optional, Dict, Tuple, List
from std_srvs.srv import Trigger, TriggerResponse


class HWTest(ABC):
    @abstractmethod
    def test_id(self) -> str:
        """Short name, used to report start and end of test"""
        pass

    @abstractmethod
    def test_desc_preparation(self) -> str:
        """Preparation before running. E.g. put the DB upside down"""
        pass

    def test_desc_running(self) -> str:
        """Actual steps to run the test"""
        # default: just click the "Run test" button
        return "Run the test"

    @abstractmethod
    def test_desc_expectation(self) -> str:
        """Expected outcome"""
        pass

    @abstractmethod
    def test_desc_log_gather(self) -> str:
        """How to gather logs before reporting"""
        pass

    def test_desc(self) -> str:
        """Test description and key params"""
        # TODO: use JSON and keys to separate sections
        return "\n\n".join([
            self.test_desc_preparation(),
            self.test_desc_expectation(),
            self.test_desc_running(),
            self.test_desc_log_gather(),
        ])

    @abstractmethod
    def run_test(self) -> Optional[bool]:  # TODO: decide whether auto grade or not
        """return True or False if the result could be determined within the test"""
        pass


class HWTestLED(HWTest):
    def __init__(self, driver: RGB_LED) -> None:
        super().__init__()
        self._driver = driver
        # test settings
        self.fade_in_secs = 2
        self.dura_secs = 10
        self.fade_out_secs = 2

    def test_id(self) -> str:
        return f"LED"

    def test_desc_preparation(self) -> str:
        return "Put your Duckiebot in normal orientation."

    def test_desc_expectation(self) -> str:
        return (
            "The Duckiebot LEDs should start shining.\n"
            "The LEDs should show smoothly: RED -> GREEN -> BLUE -> RED\n"
            f"In about {self.fade_in_secs + self.dura_secs + self.fade_out_secs} seconds, it should be off."
        )
    
    def test_desc_log_gather(self) -> str:
        return (
            "On your laptop, run the following command to save the logs.\n"
            "Replace the `[path/to/save]' to the directory path where you would like to save the logs.\n"
            "`docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt'"
        )

    def test_params(self) -> str:
        return f"[{self.test_id()}] fade_in_secs = {self.fade_in_secs}, dura_secs = {self.dura_secs}, fade_out_secs = {self.fade_out_secs}"

    def _generate_colors(self, gap: int = 16) -> List[Tuple[float, float, float]]:
        """Generate a smooth transition of colors: Red -> Green -> Blue -> Red"""
        samples = []
        # (255, g+, 0)
        for g in range(256):
            if g % gap == 0:
                samples.append((255, g, 0))
        # (r-, 255, 0)
        for r in reversed(range(256)):
            if r % gap == 0:
                samples.append((r, 255, 0))
        # (0, 255, b+)
        for b in range(256):
            if b % gap == 0:
                samples.append((0, 255, b))
        # (0, g-, 255)
        for g in reversed(range(256)):
            if g % gap == 0:
                samples.append((0, g, 255))
        # (r+, 0, 255)
        for r in range(256):
            if r % gap == 0:
                samples.append((r, 0, 255))
        # (255, 0, b-)
        for b in reversed(range(256)):
            if b % gap == 0:
                samples.append((255, 0, b))

        normalized_samples = [(float(r) / 255.0, float(g) / 255.0, float(b) / 255.0) for r, g, b in samples]
        return normalized_samples
        

    def run_test(self) -> Optional[bool]:
        num_leds = 5

        colors = self._generate_colors()
        interval = self.dura_secs / float(len(colors))

        # turn all on gradually
        n_itr = int(self.fade_in_secs / interval)
        fade_in_incr = int(255 / n_itr)
        r0 = 0
        for _ in range(n_itr):
            r0 += fade_in_incr
            r0 = min(255, r0)
            for i in range(num_leds):
                self._driver.setRGB(i, (r0 / 255.0, 0, 0))
            rospy.sleep(interval)

        for color in colors:
            for i in range(num_leds):
                self._driver.setRGB(i, color)
            rospy.sleep(interval)

        # turn all off
        n_itr = int(self.fade_out_secs / interval)
        fade_out_decr = int(255 / n_itr)
        r1 = 255
        for _ in range(n_itr):
            r1 -= fade_in_incr
            r1 = max(0, r1)
            for i in range(num_leds):
                self._driver.setRGB(i, (r1 / 255.0, 0, 0))
            rospy.sleep(interval)

        for i in range(num_leds):
            self._driver.setRGB(i, (0, 0, 0))


class LEDDriverNode(DTROS):
    """Node for controlling LEDs.

    Calls the low-level functions of class :obj:`RGB_LED` that creates the PWM
    signal used to change the color of the LEDs. The desired behavior is specified by
    the LED index (Duckiebots and watchtowers have multiple of these) and a pattern.
    A pattern is a combination of colors and blinking frequency.

    Duckiebots have 5 LEDs that are indexed and positioned as following:

    +------------------+------------------------------------------+
    | Index            | Position (rel. to direction of movement) |
    +==================+==========================================+
    | 0                | Front left                               |
    +------------------+------------------------------------------+
    | 1                | Rear left                                |
    +------------------+------------------------------------------+
    | 2                | Top / Front middle                       |
    +------------------+------------------------------------------+
    | 3                | Rear right                               |
    +------------------+------------------------------------------+
    | 4                | Front right                              |
    +------------------+------------------------------------------+

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LEDDriverNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)

        self.led = RGB_LED()

        start_color = [0, 0, 0]
        for i in range(5):
            self.led.setRGB(i, start_color)

        self.sub_topic = rospy.Subscriber("~led_pattern", LEDPattern, self.led_cb, queue_size=1)

        self.log("Initialized.")

        # hwtest
        self._desc_tst_srv = rospy.Service('~tests/led/desc', Trigger, self._tst_desc)
        self._tst_srv = rospy.Service('~tests/led/run', Trigger, self._tst)
        self._tst_def = None

    def _tst_desc(self, _):
        # this part is not in __init__ to make sure all initialization is completed
        if self._tst_def is None:
            self._tst_def = HWTestLED(self.led)
        return TriggerResponse(
            success=True,
            message=self._tst_def.test_desc(),
        )

    def _tst(self, _):
        if self._tst_def is None:
            self._tst_def = HWTestLED(self.led)
        logs = []
        success = True

        try:
            test = self._tst_def
            self.log(f"[{test.test_id()}] Started")
            self.log(test.test_params())
            logs.append(f"[{test.test_id()}] Started")
            logs.append(test.test_params())
            test.run_test()
            self.log(f"[{test.test_id()}] Finished")
            logs.append(f"[{test.test_id()}] Finished")
        except Exception as e:
            logs.append(f"Exception occured. Details: {e}")
            success = False

        return TriggerResponse(
            success=success,
            message="\n".join(logs),
        )


    def led_cb(self, msg):
        """Switches the LEDs to the requested signal."""

        for i in range(5):
            colors = [msg.rgb_vals[i].r, msg.rgb_vals[i].g, msg.rgb_vals[i].b]
            self.led.setRGB(i, colors)

    def on_shutdown(self):
        """Shutdown procedure.

        At shutdown, changes the LED pattern to `LIGHT_OFF`.
        """
        # Turn off the lights when the node dies
        self.loginfo("Shutting down. Turning LEDs off.")
        time.sleep(1)


if __name__ == "__main__":
    # Create the LEDdriverNode object
    led_driver_node = LEDDriverNode(node_name="led_driver_node")
    # Keep it spinning to keep the node alive
    rospy.spin()
