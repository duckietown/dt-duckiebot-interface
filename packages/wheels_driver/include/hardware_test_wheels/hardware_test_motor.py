import rospy

from enum import Enum, auto
from std_srvs.srv import Trigger

from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType
from wheels_driver.dagu_wheels_driver import DaguWheelsDriver


class HardwareTestMotorSide(Enum):
    LEFT = auto()
    RIGHT = auto()


class HardwareTestMotor(HardwareTest):
    def __init__(self,
                 wheel_side: "HardwareTestMotorSide",
                 motors_driver: DaguWheelsDriver,
                 vel: float = 0.5,
                 dura_secs: int = 3,
                 ) -> None:
        super().__init__()

        # attr
        self._driver = motors_driver
        self._side = wheel_side
        self._info_str = "left" if wheel_side == HardwareTestMotorSide.LEFT else "right"

        # test settings
        self.vel = vel
        self.dura_secs = dura_secs

        # test services
        self._description_srv = rospy.Service(f"~tests/{self._info_str}/description", Trigger, self.cb_description)
        self._test_srv = rospy.Service(f"~tests/{self._info_str}/run", Trigger, self._test)

    def test_id(self) -> str:
        return f"Motor ({self._info_str})"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(["Put your Duckiebot upside down."])

    def test_description_expectation(self) -> str:
        return self.html_util_ul([
            f"The {self._info_str} motor should start spinning.",
            f"In about {self.dura_secs} seconds, it should stop moving.",
        ])
    
    def test_description_log_gather(self) -> str:
        return self.html_util_ul([
            "On your laptop, run the following command to save the logs.",
            "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
            "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
        ])

    def _test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
        success = True

        try:
            start_ts = rospy.Time.now()
            end_ts = start_ts + rospy.Duration(self.dura_secs)
            if self._side == HardwareTestMotorSide.LEFT:
                self._driver.set_wheels_speed(left=self.vel, right=0.0)
            else:
                self._driver.set_wheels_speed(left=0.0, right=self.vel)
            while rospy.Time.now() < end_ts:
                rospy.sleep(1.0)
            self._driver.set_wheels_speed(left=0.0, right=0.0)
        except Exception as e:
            rospy.logerr(f"[{self.test_id()}] Experienced error: {e}")
            success = False

        params = f"[{self.test_id()}] vel = {self.vel}, dura_secs = {self.dura_secs}"

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
