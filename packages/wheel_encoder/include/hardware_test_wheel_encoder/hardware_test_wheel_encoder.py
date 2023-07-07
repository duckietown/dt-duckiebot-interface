import rospy

from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType


class HardwareTestWheelEncoder(HardwareTest):
    def __init__(self, wheel_side: str) -> None:
        super().__init__()
        # attr
        self._name = wheel_side

    def test_id(self) -> str:
        return f"Wheel Encoder ({self._name})"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(
            [
                "Put your Duckiebot upside down, where you can reach and turn the wheels by hand.",
            ]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                "Once your start the test, a <strong>Tick value</strong> field will appear below.",
                f"When you turn the {self._name} wheel, if the value changes according to your movement rate, the test is passed.",
                "(The change of directions would not be reflected.)",
            ]
        )

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")

        # Return the service response
        return self.format_response_stream(
            success=True,  # does not matter here
            test_topic_name=f"{self._name}_wheel_encoder_node/tick",
            test_topic_type="duckietown_msgs/WheelEncoderStamped",
            lst_blocks=[],
        )
