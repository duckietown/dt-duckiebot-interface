import rospy

from typing import Callable

from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType
from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg

from display_renderer import (
    MonoImageFragmentRenderer,
    PAGE_TEST_OLED_DISPLAY,
    REGION_BODY,
    DisplayROI,
)
from display_renderer.text import monospace_screen


class OLEDDisplayTestRenderer(MonoImageFragmentRenderer):
    def __init__(self, disp_text: str, duration: int):
        super(OLEDDisplayTestRenderer, self).__init__(
            name="__oled_display_test__",
            page=PAGE_TEST_OLED_DISPLAY,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
            ttl=duration,  # on shutdown, just need one fixed screen
        )

        contents = monospace_screen(
            (self.roi.h, self.roi.w), disp_text, scale="hfill", align="center"
        )
        self.data[:, :] = contents


class HardwareTestOledDisplay(HardwareTest):
    def __init__(
        self,
        fn_show_test_display: Callable[[DisplayFragmentMsg], None],
        fn_remove_test_display: Callable[[], None],
        duration: int = 5,
        disp_text: str = "OLED Display Test",
    ) -> None:
        super().__init__()
        # test settings
        self.duration = duration
        self.disp_text = disp_text
        # attr
        self._handle_start_test = fn_show_test_display
        self._handle_end_test = fn_remove_test_display

    def test_id(self) -> str:
        return "OLED Display"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(
            [
                "Place your Duckiebot on a flat surface with the top OLED screen visible.",
            ]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                f"Once the test is started, the top display should show: <strong>{self.disp_text}</strong>.",
                f"In about <strong>{self.duration}</strong> seconds, the test page should disappear, and the homepage should be shown again.",
            ]
        )

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
        success = True
        response_type = HardwareTestJsonParamType.STRING

        # Subscribe to the topic and get one message
        try:
            start_ts = rospy.Time.now()
            end_ts = start_ts + rospy.Duration(self.duration)
            # show test display
            self._handle_start_test(
                OLEDDisplayTestRenderer(
                    disp_text=self.disp_text,
                    duration=self.duration,
                ).as_msg()
            )
            # run until specified time reached
            while rospy.Time.now() < end_ts:
                rospy.sleep(1.0)
            # go back to homepage
            self._handle_end_test()
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to run the test: {e}")
            success = False

        response = f"[{self.test_id()}] duration = {self.duration}s, disp_text = '{self.disp_text}'"
        # Return the service response
        return self.format_response_object(
            success=success,
            lst_blocks=[
                self.format_obj(
                    key="Test parameters",
                    value_type=response_type,
                    value=response,
                ),
            ],
        )
