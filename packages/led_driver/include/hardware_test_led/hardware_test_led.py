import rospy
import numpy as np

from typing import List, Tuple, Dict
from colorir import HSV

from rgb_led import RGB_LED
from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType


class HardwareTestLED(HardwareTest):
    def __init__(
        self,
        driver: RGB_LED,
        info_str: str,
        led_ids: List[int],
        idle_lighting: Dict[str, List],
        fade_in_secs: int = 1,
        duration: int = 6,
        fade_out_secs: int = 1,
    ) -> None:
        # describe this group of LEDs, e.g. "front" or "back"
        self._info_str = info_str
        super().__init__(service_identifier=f"tests/{self._info_str}")

        # attr
        self._driver = driver
        self._led_ids = led_ids
        self._idle_lighting = idle_lighting  # set to this after the test

        # test settings
        self.fade_in_secs = fade_in_secs
        self.duration = duration
        self.fade_out_secs = fade_out_secs
        self._color_sequence = None  # lazy init. If test is run, generate this

    def test_id(self) -> str:
        return f"LED ({self._info_str})"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(
            [
                f"Place your Duckiebot on a flat surface in a position that allows you to see the {self._info_str} LEDs.",
            ]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                "The Duckiebot LEDs should start shining.",
                "The LEDs should show a smooth transition of these colors: "
                "RED -> YELLOW -> GREEN -> BLUE -> PURPLE -> RED.",
                f"In about {self.fade_in_secs + self.duration + self.fade_out_secs} seconds, "
                f"they should be off.",
            ]
        )

    def _generate_colors(self, step_size: int = 1) -> List[Tuple[float, float, float]]:
        """Generate a smooth transition of colors"""
        return [HSV(hue, 1.0, 1.0).rgb() for hue in range(0, 360, step_size)]

    def _fade_mono(self, fade_in: bool, interval_secs: float, mono_hue: int = 0):
        """fade IN or OUT in a mono color"""
        # number of different colors to show
        n_itr = (
            int(self.fade_in_secs / interval_secs)
            if fade_in
            else int(self.fade_out_secs / interval_secs)
        )

        # increasing brightness
        seq_v = np.arange(0.0, 1.0, float(1.0 / n_itr))
        if not fade_in:
            # or decreasing
            seq_v = reversed(seq_v)

        for v in seq_v:
            for i in self._led_ids:
                self._driver.set_RGB(i, HSV(mono_hue, 1.0, v).rgb(), is_test_cmd=True)
            rospy.sleep(interval_secs)

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
        success = True

        # generate test color sequence if not yet initialized
        if self._color_sequence is None:
            self._color_sequence = self._generate_colors()

        interval = self.duration / float(len(self._color_sequence))

        try:
            self._driver.start_hardware_test()
            # turn all on gradually
            self._fade_mono(fade_in=True, interval_secs=interval)

            # run color sequence test
            for color in self._color_sequence:
                for i in self._led_ids:
                    self._driver.set_RGB(i, color, is_test_cmd=True)
                rospy.sleep(interval)

            # turn all off
            self._fade_mono(fade_in=False, interval_secs=interval)

            # make sure they are set to idle lighting
            for i in self._led_ids:
                self._driver.set_RGB(
                    led=i,
                    color=self._idle_lighting["color"][i],
                    intensity=self._idle_lighting["intensity"][i],
                    is_test_cmd=True,
                )
        except Exception as e:
            rospy.logerr(f"[{self.test_id()}] Experienced error: {e}")
            success = False
        finally:
            self._driver.finish_hardware_test()

        params = f"[{self.test_id()}] fade_in_secs = {self.fade_in_secs}s, duration = {self.duration}s, fade_out_secs = {self.fade_out_secs}s"

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
