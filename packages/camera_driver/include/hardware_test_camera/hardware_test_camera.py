import rospy

from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType


class HardwareTestCamera(HardwareTest):
    def __init__(self) -> None:
        super().__init__()

    def test_id(self) -> str:
        return "Camera"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(
            ["Place the Duckiebot within your reach, as you will need to move it during the test."]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(["A live camera stream should be shown below."])

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")

        instructions = self.html_util_ul(
            [
                "Move you Duckiebot for a few seconds, and check the stream corresponds to that.",
            ]
        )

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
