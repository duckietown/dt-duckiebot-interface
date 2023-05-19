import base64
import rospy

from std_srvs.srv import Trigger
from dt_duckiebot_hardware_tests import HWTest, HWTestJsonParamType


class HWTestCamera(HWTest):
    def __init__(self) -> None:
        super().__init__()
        # test services
        self._desc_tst_srv = rospy.Service('~test/desc', Trigger, self.srv_cb_tst_desc)
        self._tst_srv = rospy.Service('~test/run', Trigger, self._tst)

    def test_id(self) -> str:
        return "Camera"

    def test_desc_preparation(self) -> str:
        return self.html_util_ul(["Put the Duckiebot within your reach, as you need to move it."])

    def test_desc_expectation(self) -> str:
        return self.html_util_ul(["A live camera stream will be shown below."])

    def test_desc_log_gather(self) -> str:
        return self.html_util_ul([
            "On your laptop, run the following command to save the logs.",
            "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
            "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
        ])

    def _tst(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
    
        instructions = self.html_util_ul([
            "Move you Duckiebot for a few seconds, and check the stream corresponds to that.",
        ])

        # Return the service response
        return self.format_response_stream(
            success=True,  # does not matter here
            test_topic_name=f"camera_node/image/compressed",
            test_topic_type="sensor_msgs/CompressedImage",
            lst_blocks=[
                self.format_obj(
                    key="Instructions",
                    value_type=HWTestJsonParamType.HTML,
                    value=instructions,
                ),
            ],
        )
