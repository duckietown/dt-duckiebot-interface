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

    ### Below is the old code for a preliminary test, the code is kept for reference
    ### It asks the user to tilt the robot and use 2 different messages of IMU to determine the motion

    # # Instructions for preliminary test: i.e. only tilting and let program tell the direction
    # def test_desc_running(self) -> str:
    #     return (
    #         "Put your hand near the Duckiebot\n"
    #         "Once the test starts, you should tilt the robot left/right, i.e. make the top display face your left/right. And the robot will be standing on the side of either left or right wheel.\n"
    #         "Now, click the button below to run the test"
    #     )

    # def test_desc_expectation(self) -> str:
    #     return (
    #         f"The experiment will finish in {self.dura_secs} seconds.\n"
    #         "After finishing, the response string below should indicate, whether the robot is inclined to the left or the right.\n"
    #         "You should run the test MULTIPLE times (left, standing still, right), to verify."
    #     )

    # # The preliminary test
    # def _tst(self, _):
    #     msg_start = None
    #     msg_end = None

    #     # Subscribe to the topic and get one message
    #     try:
    #         # initial measurement
    #         msg_start = rospy.wait_for_message('~imu_data', Imu, timeout=1.0)
    #         success = True
    #     except rospy.ROSException as e:
    #         success = False

    #     rospy.sleep(self.dura_secs)

    #     if success:
    #         try:
    #             # final measurement
    #             msg_end = rospy.wait_for_message('~imu_data', Imu, timeout=1.0)
    #             success = True
    #         except rospy.ROSException as e:
    #             success = False

    #     if success:
    #         dx = msg_start.linear_acceleration.x - msg_end.linear_acceleration.x
    #         response = "No inclination along x axis detected."
    #         if abs(dx) > self.abs_min_incline_d_accel:
    #             if dx < 0:
    #                 response = "Inclined to the left (<-)"
    #             else:
    #                 response = "Inclined to the right (->)"
    #     else:
    #         response = "Failed to take IMU measurements"

    #     # Return the service response
    #     return TriggerResponse(success=success, message=response)
