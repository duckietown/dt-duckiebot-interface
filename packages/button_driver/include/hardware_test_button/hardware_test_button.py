import rospy

from button_driver import ButtonDriver
from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType


class HardwareTestButton(HardwareTest):
    def __init__(
        self,
        driver: ButtonDriver,
        led_blink_secs: int = 3,
        led_blink_hz: int = 1,
    ) -> None:
        super().__init__()

        # attr
        self._driver = driver
        self._button_released = False

        # test settings
        self.led_blink_secs = led_blink_secs
        self.led_blink_hz = led_blink_hz

    def test_id(self) -> str:
        return "Top button"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(
            [
                "Place your Duckiebot on a flat surface and locate the power button on the top plate."
            ]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                f"You should see the LED in the power button blink for about {self.led_blink_secs} seconds.",
                "When the LED <strong>stops</strong> blinking, press and release the button to complete the test.",
                "Try performing this test several times to verify that pressing the power button terminates the test promptly.",
            ]
        )

    def button_event_cb(self):
        self._button_released = True

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
        success = True

        try:
            # button led test
            self._driver.led.blink_led(
                secs_to_blink=self.led_blink_secs,
                blink_freq_hz=self.led_blink_hz,
            )
            # button press event test
            self._driver.start_test(self.button_event_cb)
            while not self._button_released:
                rospy.sleep(0.1)
            # reset
            self._button_released = False
        except Exception as e:
            rospy.logerr(f"[{self.test_id()}] Experienced error: {e}")
            success = False

        params = f"[{self.test_id()}] led_blink_secs = {self.led_blink_secs}, led_blink_hz = {self.led_blink_hz}"

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
