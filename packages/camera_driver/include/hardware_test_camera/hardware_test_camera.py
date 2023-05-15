import base64
import rospy

from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage
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
        return self.html_util_ul([
            "Point the camera to a place with no confidential data, as we will take a picture during the test.",
            "The picture taken will not be stored anywhere.",
        ])

    def test_desc_expectation(self) -> str:
        return self.html_util_ul([
            "Run the test a few times, with slightly different Duckiebot headings.",
            "The pictures taken should correspond to the Duckiebot's view change",
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
        response = ""
        response_type = HWTestJsonParamType.BASE64
    
        # Subscribe to the topic and get one message
        try:
            msg = rospy.wait_for_message('~image/compressed', CompressedImage, timeout=5.0)
            base64_encoded_msg = base64.b64encode(msg.data).decode('utf-8')
            response = base64_encoded_msg
        except rospy.ROSException as e:
            rospy.logerr(f"[{self.test_id()}] Experienced error: {e}")
            success = False

        # Return the service response
        return self.format_response_object(
            success=success,
            lst_blocks=[
                self.format_obj(
                    key="Image",
                    value_type=response_type,
                    value=response,
                ),
            ]
        )
