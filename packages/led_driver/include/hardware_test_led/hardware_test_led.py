import rospy

from typing import List, Tuple

from rgb_led import RGB_LED
from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType


class HardwareTestLED(HardwareTest):
    def __init__(
        self,
        driver: RGB_LED,
        info_str: str,
        led_ids: List[int],
        fade_in_secs: int = 2,
        dura_secs: int = 10,
        fade_out_secs: int = 2,
    ) -> None:
        # describe this group of LEDs, e.g. "front" or "back"
        self._info_str = info_str
        super().__init__(service_identifier=f"tests/{self._info_str}")

        # attr
        self._driver = driver
        self._led_ids = led_ids

        # test settings
        self.fade_in_secs = fade_in_secs
        self.dura_secs = dura_secs
        self.fade_out_secs = fade_out_secs
        self._color_sequence = None  # lazy init. If test is run, generate this

    def test_id(self) -> str:
        return f"LED ({self._info_str})"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(
            [
                f"Put your Duckiebot in its normal orientation, where you can see the {self._info_str} LEDs.",
            ]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                "The Duckiebot LEDs should start shining.",
                "The LEDs should show a smooth transition of these colors: RED -> GREEN -> BLUE -> RED.",
                f"In about {self.fade_in_secs + self.dura_secs + self.fade_out_secs} seconds, they should be off.",
            ]
        )

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

        normalized_samples = [
            (float(r) / 255.0, float(g) / 255.0, float(b) / 255.0)
            for r, g, b in samples
        ]
        return normalized_samples

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
        success = True

        # generate test color sequence if not yet initialized
        if self._color_sequence is None:
            self._color_sequence = self._generate_colors()

        interval = self.dura_secs / float(len(self._color_sequence))

        try:
            self._driver.start_hardware_test()
            # turn all on gradually
            n_itr = int(self.fade_in_secs / interval)
            fade_in_incr = int(255 / n_itr)
            r0 = 0
            for _ in range(n_itr):
                r0 += fade_in_incr
                r0 = min(255, r0)
                for i in self._led_ids:
                    self._driver.set_RGB(i, (r0 / 255.0, 0, 0), is_test_cmd=True)
                rospy.sleep(interval)

            # run color sequence test
            for color in self._color_sequence:
                for i in self._led_ids:
                    self._driver.set_RGB(i, color, is_test_cmd=True)
                rospy.sleep(interval)

            # turn all off
            n_itr = int(self.fade_out_secs / interval)
            fade_out_decr = int(255 / n_itr)
            r1 = 255
            for _ in range(n_itr):
                r1 -= fade_out_decr
                r1 = max(0, r1)
                for i in self._led_ids:
                    self._driver.set_RGB(i, (r1 / 255.0, 0, 0), is_test_cmd=True)
                rospy.sleep(interval)

            # make sure they are off
            for i in self._led_ids:
                self._driver.set_RGB(i, (0, 0, 0), is_test_cmd=True)
        except Exception as e:
            rospy.logerr(f"[{self.test_id()}] Experienced error: {e}")
            success = False
        finally:
            self._driver.finish_hardware_test()

        params = f"[{self.test_id()}] fade_in_secs = {self.fade_in_secs}, dura_secs = {self.dura_secs}, fade_out_secs = {self.fade_out_secs}"

        return self.format_response_object(
            success=success,
            lst_blocks=[
                self.format_obj(
                    key="Test parameters",
                    value_type=HardwareTestJsonParamType.STRING,
                    value=params,
                ),
            ],
        )
