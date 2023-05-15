import rospy

from std_srvs.srv import Trigger
from dt_duckiebot_hardware_tests import HWTest, HWTestJsonParamType


class HWTestWheelEncoder(HWTest):
    def __init__(self, wheel_side: str) -> None:
        super().__init__()
        # test settings

        # attr
        self._name = wheel_side

        # test services
        self._desc_tst_srv = rospy.Service('~test/desc', Trigger, self.srv_cb_tst_desc)
        self._tst_srv = rospy.Service('~test/run', Trigger, self._tst)
    
    def test_id(self) -> str:
        return f"Wheel Encoder ({self._name})"

    def test_desc_preparation(self) -> str:
        return self.html_util_ul([
            "Put your Duckiebot upside-down, where you can reach and turn the wheels by hand."
        ])

    def test_desc_expectation(self) -> str:
        return self.html_util_ul([
            "Once your start the test, a <strong>Tick value</strong> field will appear below.",
            "When you turn the wheel, if the tick value below changes according to your movements and rate, the test is passed."
        ])

    def test_desc_log_gather(self) -> str:
        return self.html_util_ul([
            "On your laptop, run the following command to save the logs.",
            "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
            "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
        ])

    def _tst(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")

        instructions = self.html_util_ul([
            f"Now spin the <strong>{self._name}</strong> wheel by hand.",
            "Once your decide the test has passed/failed, you may mark the decision, and close this modal.",
        ])

        # Return the service response
        return self.format_response_stream(
            success=True,  # does not matter here
            test_topic_name=f"{self._name}_wheel_encoder_node/tick",
            test_topic_type="duckietown_msgs/WheelEncoderStamped",
            lst_blocks=[
                self.format_obj(
                    key="Instructions",
                    value_type=HWTestJsonParamType.HTML,
                    value=instructions,
                ),
            ],
        )
