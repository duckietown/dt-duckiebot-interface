import rospy

from std_srvs.srv import Trigger
from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType

from tof_accuracy import ToFAccuracy


class HardwareTestToF(HardwareTest):
    def __init__(self, sensor_name: str, accuracy: ToFAccuracy) -> None:
        super().__init__()
        # test settings

        # attr
        self._sensor_name = sensor_name  # e.g. front_center
        self._accuracy = accuracy

        # test services
        self._description_srv = rospy.Service('~test/description', Trigger, self.cb_description)
        self._test_srv = rospy.Service('~test/run', Trigger, self._test)

    def test_id(self) -> str:
        return f"Time-of-Flight ({self._sensor_name})"

    def test_description_preparation(self) -> str:
        return self.html_util_ul([
            "Put your Duckiebot in its normal orientation.",
            "Place it near you, with its camera facing an empty space.",
        ])

    def test_description_expectation(self) -> str:
        return self.html_util_ul([
            "Once your start the test, a <strong>Range:</strong> field will appear below.",
            f"When you move your hand closer and farther to the {self._sensor_name} ToF, the range reading should change accordingly, i.e. moving closer leads to a smaller value, and farther with a greater value.",
            f"The effective range of the sensor is <strong>from {self._accuracy.min_range}m to {self._accuracy.max_range}m</strong>. The data below should show (in red color) <em>Out of range</em>."
        ])

    def test_description_log_gather(self) -> str:
        return self.html_util_ul([
            "On your laptop, run the following command to save the logs.",
            "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
            "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
        ])

    def _test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")

        instructions = self.html_util_ul([
            f"Now move your hand in front of the <strong>{self._sensor_name}</strong> Time-of-Flight sensor at different distances.",
            "Once your decide the test has passed/failed, you may mark the decision, and close this modal.",
        ])

        # Return the service response
        return self.format_response_stream(
            success=True,  # does not matter here
            test_topic_name=f"{self._sensor_name}_tof_driver_node/range",
            test_topic_type="sensor_msgs/Range",
            lst_blocks=[
                self.format_obj(
                    key="Instructions",
                    value_type=HardwareTestJsonParamType.HTML,
                    value=instructions,
                ),
            ],
        )
