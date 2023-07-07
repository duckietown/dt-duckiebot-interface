import rospy

from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType

from tof_accuracy import ToFAccuracy


class HardwareTestToF(HardwareTest):
    def __init__(self, sensor_name: str, accuracy: ToFAccuracy) -> None:
        super().__init__()
        # attr
        self._sensor_name = sensor_name  # e.g. "front_center"
        self._sensor_name_human = sensor_name.replace("_", " ")  # e.g. "front center"
        self._accuracy = accuracy

    def test_id(self) -> str:
        return f"Time-of-Flight ({self._sensor_name})"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(
            [
                "Place your Duckiebot on a flat surface with at least one meter of empty space in front of the Time-of-Flight sensor on the front bumper.",
            ]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                "Once your start the test, a <strong>Range:</strong> field will appear below.",
                f"When you move your hand closer and farther to the <strong>{self._sensor_name_human}</strong> ToF, the range reading should change accordingly, i.e. moving closer leads to a smaller value, and farther with a greater value.",
                f"The effective range of the sensor is <strong>from {self._accuracy.min_range}m to {self._accuracy.max_range}m</strong>. The data below should show (in red color) <em>Out of range</em>.",
            ]
        )

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")

        # Return the service response
        return self.format_response_stream(
            success=True,  # does not matter here
            test_topic_name=f"{self._sensor_name}_tof_driver_node/range",
            test_topic_type="sensor_msgs/Range",
            lst_blocks=[],
        )
