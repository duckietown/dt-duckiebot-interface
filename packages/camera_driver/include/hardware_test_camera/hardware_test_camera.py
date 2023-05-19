import base64
import rospy

from std_srvs.srv import Trigger
from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType


class HardwareTestCamera(HardwareTest):
    def __init__(self) -> None:
        super().__init__()
        # test services
        self._description_srv = rospy.Service('~test/description', Trigger, self.cb_description)
        self._test_srv = rospy.Service('~test/run', Trigger, self._test)

    def test_id(self) -> str:
        return "Camera"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(["Put the Duckiebot within your reach, as you need to move it."])

    def test_description_expectation(self) -> str:
        return self.html_util_ul(["A live camera stream will be shown below."])

    def test_description_log_gather(self) -> str:
        return self.html_util_ul([
            "On your laptop, run the following command to save the logs.",
            "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
            "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
        ])

    def _test(self, _):
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
                    value_type=HardwareTestJsonParamType.HTML,
                    value=instructions,
                ),
            ],
        )
