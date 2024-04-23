#!/usr/bin/env python3
import asyncio
import time

import argparse
import dataclasses
from typing import Type, Optional

from dt_node_utils.decorators import sidecar
from dtps_http import RawData

from dtps import DTPSContext

from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.node import Node
from dt_robot_utils import RobotHardware, get_robot_hardware
from duckietown_messages.actuators.differential_pwm import DifferentialPWM
from duckietown_messages.standard.boolean import Boolean
from duckietown_messages.utils.exceptions import DataDecodingError
from wheels_driver.wheels_driver_abs import WheelsDriverAbs, WheelPWMConfiguration

if get_robot_hardware() == RobotHardware.VIRTUAL:
    from wheels_driver.virtual_wheels_driver import VirtualWheelsDriver
    WheelsDriver: Type[WheelsDriverAbs] = VirtualWheelsDriver
else:
    from wheels_driver.wheels_driver import DaguWheelsDriver
    WheelsDriver: Type[WheelsDriverAbs] = DaguWheelsDriver


@dataclasses.dataclass
class WheelsDriverNodeConfiguration(NodeConfiguration):
    pwm_left: WheelPWMConfiguration
    pwm_right: WheelPWMConfiguration
    safety_stop_timeout: float


class WheelsDriverNode(Node):
    """Node handling the motor speeds.

    Subscribes to the requested wheels commands (linear velocities, i.e. velocity for the left
    and the right wheels) and to an emergency stop flag.
    When the emergency flag `~emergency_stop` is set to `False` it actuates the wheel driver
    with the velocities received from `~wheels_cmd`. Publishes the execution of the commands
    to `~wheels_cmd_executed`.

    The emergency stop is `False` by default.

    """

    def __init__(self, config: str, actuator_name: str):
        node_name: str = f"wheels-driver-{actuator_name}"
        super(WheelsDriverNode, self).__init__(
            name=node_name,
            kind=NodeType.DRIVER,
            description="Robot wheels motor driver",
        )
        self.actuator_name: str = actuator_name

        # load configuration
        self.configuration: WheelsDriverNodeConfiguration = (WheelsDriverNodeConfiguration.
                                                             from_name(self.package, node_name, config))
        # emergency stop
        self.estop: bool = False
        # keep track of when the last command was received
        self.last_command_time: float = 0.0
        # queues
        self._pwm_filtered_out: Optional[DTPSContext] = None
        self._pwm_executed_out: Optional[DTPSContext] = None
        # setup the driver
        self.driver: WheelsDriverAbs = WheelsDriver(
            left_config=WheelPWMConfiguration(),
            right_config=WheelPWMConfiguration(),
        )

    async def cb_wheels_pwm(self, data: RawData):
        """
        Callback that sets wheels' PWM signals.
        """
        try:
            pwms: DifferentialPWM = DifferentialPWM.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        pwm_left: float = 0.0
        pwm_right: float = 0.0
        self.last_command_time = time.time()
        if not self.estop:
            pwm_left = pwms.left
            pwm_right = pwms.right

        self.driver.set_wheels_speed(left=pwm_left, right=pwm_right)
        # publish out the filtered signals
        filtered: DifferentialPWM = DifferentialPWM(left=pwm_left, right=pwm_right)
        await self._pwm_filtered_out.publish(filtered.to_rawdata())
        # publish out the executed signals
        executed: DifferentialPWM = DifferentialPWM(left=self.driver.left_pwm, right=self.driver.right_pwm)
        await self._pwm_executed_out.publish(executed.to_rawdata())

    async def cb_estop(self, data: RawData):
        """
        Callback that enables/disables emergency stop.
        """
        try:
            msg: Boolean = Boolean.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # ---
        self.estop = msg.data
        if self.estop:
            self.loginfo("Emergency Stop Activated")
        else:
            self.loginfo("Emergency Stop Released")

    async def worker(self):
        await self.dtps_init(self.configuration)
        # create PWM queue IN
        pwm_in: DTPSContext = await (self.context / "in" / "pwm").queue_create()
        # create emergency stop queue
        estop_queue: DTPSContext = await (self.context / "in" / "estop").queue_create()
        # create PWM queues OUT
        self._pwm_filtered_out: DTPSContext = await (self.context / "out" / "pwm_filtered").queue_create()
        self._pwm_executed_out: DTPSContext = await (self.context / "out" / "pwm_executed").queue_create()
        # subscribe to PWM commands
        await pwm_in.subscribe(self.cb_wheels_pwm)
        # subscribe to emergency stop commands
        await estop_queue.subscribe(self.cb_estop)
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        actuator: DTPSContext = self.switchboard / "actuator" / "wheels" / self.actuator_name
        await (actuator / "pwm").expose(pwm_in)
        await (actuator / "estop").expose(estop_queue)
        await (actuator / "pwm_filtered").expose(self._pwm_filtered_out)
        await (actuator / "pwm_executed").expose(self._pwm_executed_out)
        # publish the initial state
        await pwm_in.publish(DifferentialPWM(left=0, right=0).to_rawdata())
        await estop_queue.publish(Boolean(data=False).to_rawdata())
        # run forever
        await self.join()

    @sidecar
    async def safety_stop(self):
        holding: bool = False
        while not self.is_shutdown:
            # stop wheels if no commands are received for a while
            if time.time() - self.last_command_time > self.configuration.safety_stop_timeout:
                if not holding:
                    self.driver.set_wheels_speed(0, 0)
                holding = True
            else:
                holding = False
            await asyncio.sleep(0.5)

    def on_shutdown(self):
        """
        Shutdown procedure.

        Publishes a zero velocity command at shutdown.
        """
        self.driver.set_wheels_speed(left=0.0, right=0.0)


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--actuator-name", type=str, required=True, help="Name of the actuator")
    parser.add_argument("--config", type=str, required=True, help="Name of the configuration")
    args: argparse.Namespace = parser.parse_args()
    # create node
    node: WheelsDriverNode = WheelsDriverNode(config=args.config, actuator_name=args.actuator_name)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
