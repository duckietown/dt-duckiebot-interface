import base64
import rospy

from typing import Callable

from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage
from dt_duckiebot_hardware_tests import HWTest, HWTestJsonParamType
from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg

from display_renderer import (
    MonoImageFragmentRenderer,
    PAGE_TEST_OLED_DISPLAY,
    REGION_BODY,
    DisplayROI,
)
from display_renderer.text import monospace_screen

class OLEDDisplayTestRenderer(MonoImageFragmentRenderer):
    def __init__(self, disp_text: str, dura_secs: int):
        super(OLEDDisplayTestRenderer, self).__init__(
            name=f"__oled_display_test__",
            page=PAGE_TEST_OLED_DISPLAY,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
            ttl=dura_secs,  # on shutdown, just need one fixed screen
        )

        contents = monospace_screen((self.roi.h, self.roi.w), disp_text, scale="hfill", align="center")
        self.data[:, :] = contents


class HWTestOledDisplay(HWTest):
    def __init__(self,
                 fn_show_test_display: Callable[[DisplayFragmentMsg], None],
                 fn_remove_test_display: Callable[[], None],
                 dura_secs: int = 5,
                 disp_text: str = "OLED Display Test",
                 ) -> None:
        super().__init__()
        # test settings
        self.dura_secs = dura_secs
        self.disp_text = disp_text
        # attr
        self._handle_start_test = fn_show_test_display
        self._handle_end_test = fn_remove_test_display
        # test services
        self._desc_tst_srv = rospy.Service('~test/desc', Trigger, self.srv_cb_tst_desc)
        self._tst_srv = rospy.Service('~test/run', Trigger, self._tst)

    def test_id(self) -> str:
        return "OLED Display"

    def test_desc_preparation(self) -> str:
        return self.html_util_ul([
            "Put your Duckiebot in normal orientation.",
            "And make sure you can see the top OLED display.",
        ])

    def test_desc_expectation(self) -> str:
        return self.html_util_ul([
            f"The top display should show: <strong>{self.disp_text}</strong>.",
            f"In about <strong>{self.dura_secs}</strong> seconds, the test page should disappear, and the homepage should be shown again.",
        ])

    def test_desc_log_gather(self) -> str:
        return self.html_util_ul([
            "On your laptop, run the following command to save the logs.",
            "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
            "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
        ])

    def _tst(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
        success = True
        response_type = HWTestJsonParamType.STRING
    
        # Subscribe to the topic and get one message
        try:
            start_ts = rospy.Time.now()
            end_ts = start_ts + rospy.Duration(self.dura_secs)
            # show test display
            self._handle_start_test(OLEDDisplayTestRenderer(
                disp_text=self.disp_text,
                dura_secs=self.dura_secs,
            ).as_msg())
            # run until specified time reached
            while rospy.Time.now() < end_ts:
                rospy.sleep(1.0)
            # go back to homepage
            self._handle_end_test()
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to run the test: {e}")
            success = False

        response = f"[{self.test_id()}] dura_secs = {self.dura_secs}, disp_text = '{self.disp_text}'"
        # Return the service response
        return self.format_response_object(
            success=success,
            lst_blocks=[
                self.format_obj(
                    key="Test parameters",
                    value_type=response_type,
                    value=response,
                ),
            ]
        )
