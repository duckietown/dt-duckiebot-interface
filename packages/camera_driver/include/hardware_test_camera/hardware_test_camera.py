import rospy

from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType


class HardwareTestCamera(HardwareTest):
    def __init__(self) -> None:
        super().__init__()

    def test_id(self) -> str:
        return "Camera"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(
            [
                "Place the Duckiebot on a flat surface within your reach, and make sure that the lens cap has been removed from the camera."
            ]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                "You should see a live camera stream from your Duckiebot displayed below.",
                "Move your Duckiebot in a few directions and confirm that the camera stream is updating with the camera motion.",
            ]
        )

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")

        # Return the service response
        return self.format_response_stream(
            success=True,  # does not matter here
            test_topic_name=f"camera_node/image/compressed",
            test_topic_type="sensor_msgs/CompressedImage",
            lst_blocks=[],
        )
