import rospy

from std_srvs.srv import Trigger
from dt_duckiebot_hardware_tests import HWTest, HWTestJsonParamType

class HWTestIMU(HWTest):
    def __init__(self) -> None:
        super().__init__()
        # test services
        self._desc_tst_srv = rospy.Service('~test/desc', Trigger, self.srv_cb_tst_desc)
        self._tst_srv = rospy.Service('~test/run', Trigger, self._tst)

    def test_id(self) -> str:
        return f"IMU"

    def test_desc_preparation(self) -> str:
        return self.html_util_ul([
            "Put your Duckiebot in the ordinary orientation, on a surface (e.g. desk).",
            "Make sure you can reach and grab the robot when the test starts.",
        ])

    def test_desc_running(self) -> str:
        return self.html_util_ul([
            "Hold your Duckiebot up a little bit off the surface.",
            "In the next screen, your objective is to keep a ball on the plane by rotating the robot.",
            "Now, click on the <strong>Run the test</strong> button below to start the test.",
        ])

    def test_desc_expectation(self) -> str:
        return self.html_util_ul([
            "Once the test starts, you will see a flat plane reflecting your Duckiebot's orientation.",
            "If the plane does not move like your Duckiebot, there is a problem.",
            "The test is considered successful, if you're able to control the plane naturally with moving your Duckiebot.",
        ])

    
    def test_desc_log_gather(self) -> str:
        return self.html_util_ul([
            "On your laptop, run the following command to save the logs.",
            "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
            "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
            "Also, right click in the web browser and choose the `Inspect' option.",
            "Then, navigate to the `Console' tab and copy any error messages.",
        ])

    def _tst(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")

        # Return the service response
        return self.format_response_stream(
            success=True,  # does not matter here
            test_topic_name=f"imu_node/imu_data",
            test_topic_type="sensor_msgs/Imu",
            lst_blocks=[
                self.format_obj(
                    key="Success Criterion",
                    value_type=HWTestJsonParamType.STRING,
                    value="Did the plane move according to your Duckiebot movements?",
                ),
            ],
        )
