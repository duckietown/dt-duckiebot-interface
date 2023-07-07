import rospy

from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType


class HardwareTestIMU(HardwareTest):
    def __init__(self) -> None:
        super().__init__()

    def test_id(self) -> str:
        return f"IMU"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(
            [
                "Place your Duckiebot on a flat surface within reach.",
            ]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                "Once the test starts, a modal will pop up with a game.",
                "You will see a flat plane reflecting your Duckiebot's orientation - your objective is to move the ball while keeping it on the plane.",
                "To play, control the plane by <strong>picking up and tilting</strong> your Duckiebot.",
                "The test is successful if the plane moves naturally with the rotation of your Duckiebot.",
            ]
        )

    def test_description_log_gather(self) -> str:
        return self.html_util_ul(
            [
                "On your laptop, run the following command to save the logs.",
                "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
                "<code>docker -H [ROBOT_NAME].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
                "Also, right click in the web browser and choose the `Inspect' option.",
                "Then, navigate to the `Console' tab and copy any error messages.",
            ]
        )

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")

        # Return the service response
        return self.format_response_stream(
            success=True,  # does not matter here
            test_topic_name=f"imu_node/data",
            test_topic_type="sensor_msgs/Imu",
            lst_blocks=[
                self.format_obj(
                    key="Success Criterion",
                    value_type=HardwareTestJsonParamType.STRING,
                    value="Did the plane move according to your Duckiebot movements?",
                ),
            ],
        )
