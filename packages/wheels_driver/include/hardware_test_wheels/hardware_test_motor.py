import rospy

from enum import Enum, auto

from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType
from wheels_driver.dagu_wheels_driver import DaguWheelsDriver


class HardwareTestMotorSide(Enum):
    LEFT = auto()
    RIGHT = auto()


class HardwareTestMotor(HardwareTest):
    def __init__(
        self,
        wheel_side: "HardwareTestMotorSide",
        motors_driver: DaguWheelsDriver,
        vel: float = 0.5,
        duration: int = 3,
    ) -> None:
        self._info_str = "left" if wheel_side == HardwareTestMotorSide.LEFT else "right"
        super().__init__(service_identifier=f"tests/{self._info_str}")

        # attr
        self._driver = motors_driver
        self._side = wheel_side

        # test settings
        self.vel = vel
        self.duration = duration

    def test_id(self) -> str:
        return f"Motor ({self._info_str})"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(
            [
                "Place your Duckiebot upside down on a flat surface with its wheels in the air.",
                "When you run the test, the wheels will start to spin.",
            ]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                f"The {self._info_str} motor should start spinning.",
                f"In about {self.duration} seconds, it should stop moving.",
            ]
        )

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
        success = True

        try:
            self._driver.start_hardware_test()
            start_ts = rospy.Time.now()
            end_ts = start_ts + rospy.Duration(self.duration)
            if self._side == HardwareTestMotorSide.LEFT:
                self._driver.set_wheels_speed(
                    left=self.vel, right=0.0, is_test_cmd=True
                )
            else:
                self._driver.set_wheels_speed(
                    left=0.0, right=self.vel, is_test_cmd=True
                )
            while rospy.Time.now() < end_ts:
                rospy.sleep(1.0)
            self._driver.set_wheels_speed(left=0.0, right=0.0, is_test_cmd=True)
        except Exception as e:
            rospy.logerr(f"[{self.test_id()}] Experienced error: {e}")
            success = False
        finally:
            self._driver.finish_hardware_test()

        params = f"[{self.test_id()}] vel = {self.vel}, duration = {self.duration}s"

        return self.format_response_object(
            success=success,
            lst_blocks=[
                self.format_obj(
                    key="Test parameters",
                    value_type=HardwareTestJsonParamType.STRING,
                    value=params,
                ),
            ],
        )
