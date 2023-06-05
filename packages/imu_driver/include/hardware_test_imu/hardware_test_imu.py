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
                "Put your Duckiebot in its normal orientation, on a surface (e.g. desk).",
                "Make sure you can reach and grab the robot when the test starts.",
            ]
        )

    def test_description_running(self) -> str:
        return self.html_util_ul(
            [
                "Hold your Duckiebot up a little bit off the surface.",
                "Click on the <strong>Run the test</strong> button below to start the test.",
            ]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                "Once the test starts, a modal would pop up with a game.",
                "You will see a flat plane reflecting your Duckiebot's orientation.",
                "Your objective is to keep a ball on the plane by rotating your robot.",
                "The test is considered successful, if you're able to control the plane naturally by moving your Duckiebot.",
                "If the plane does not move like your Duckiebot, there is a problem.",
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
