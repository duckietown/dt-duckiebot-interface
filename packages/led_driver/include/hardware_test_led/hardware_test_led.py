import rospy

from typing import List, Tuple
from std_srvs.srv import Trigger

from rgb_led import RGB_LED
from duckietown.dtros import DTROS, TopicType, NodeType, HWTest, HWTestJsonParamType


class HWTestLED(HWTest):
    def __init__(self,
                 driver: RGB_LED,
                 front_leds: bool = True,
                 fade_in_secs: int = 2,
                 dura_secs: int = 10,
                 fade_out_secs: int = 2,
                 ) -> None:
        super().__init__()

        # attr
        self._driver = driver
        self._testing_front_leds = front_leds  # if false, testing back LEDs
        self._info_str = "front" if front_leds else "back"
        # tested on DB21J: front - [0, 2]; back - [3, 4]
        self._led_ids = [0, 1, 2] if front_leds else [3, 4]

        # test settings
        self.fade_in_secs = 2
        self.dura_secs = 10
        self.fade_out_secs = 2
        self._color_sequence = None  # lazy init. If test is run, generate this

        # test services
        self._desc_tst_srv = rospy.Service(f"~test/{self._info_str}/desc", Trigger, self.srv_cb_tst_desc)
        self._tst_srv = rospy.Service(f"~test/{self._info_str}/run", Trigger, self._tst)

    def test_id(self) -> str:
        return f"LED ({self._info_str})"

    def test_desc_preparation(self) -> str:
        return self.html_util_ul([
            f"Put your Duckiebot in ordinary orientation, where you can see the {self._info_str} LEDs.",
        ])

    def test_desc_expectation(self) -> str:
        return self.html_util_ul([
            "The Duckiebot LEDs should start shining.",
            "The LEDs should show smoothly: RED -> GREEN -> BLUE -> RED.",
            f"In about {self.fade_in_secs + self.dura_secs + self.fade_out_secs} seconds, it should be off.",
        ])
    
    def test_desc_log_gather(self) -> str:
        return self.html_util_ul([
            "On your laptop, run the following command to save the logs.",
            "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
            "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
        ])

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
        
    def _tst(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
        success = True

        # generate test color sequence if not yet initialized
        if self._color_sequence is None:
            self._color_sequence = self._generate_colors()

        interval = self.dura_secs / float(len(self._color_sequence))

        try:
            # turn all on gradually
            n_itr = int(self.fade_in_secs / interval)
            fade_in_incr = int(255 / n_itr)
            r0 = 0
            for _ in range(n_itr):
                r0 += fade_in_incr
                r0 = min(255, r0)
                for i in self._led_ids:
                    self._driver.setRGB(i, (r0 / 255.0, 0, 0))
                rospy.sleep(interval)

            # run color sequence test
            for color in self._color_sequence:
                for i in self._led_ids:
                    self._driver.setRGB(i, color)
                rospy.sleep(interval)

            # turn all off
            n_itr = int(self.fade_out_secs / interval)
            fade_out_decr = int(255 / n_itr)
            r1 = 255
            for _ in range(n_itr):
                r1 -= fade_out_decr
                r1 = max(0, r1)
                for i in self._led_ids:
                    self._driver.setRGB(i, (r1 / 255.0, 0, 0))
                rospy.sleep(interval)

            # make sure they are off
            for i in self._led_ids:
                self._driver.setRGB(i, (0, 0, 0))
        except Exception as e:
            rospy.logerror(f"[{self.test_id()}] Experienced error: {e}")
            success = False

        params = f"[{self.test_id()}] fade_in_secs = {self.fade_in_secs}, dura_secs = {self.dura_secs}, fade_out_secs = {self.fade_out_secs}"

        return self.format_response_object(
            success=success,
            lst_blocks=[
                self.format_obj(
                    key="Test parameters",
                    value_type=HWTestJsonParamType.STRING,
                    value=params,
                ),
            ],
        )
