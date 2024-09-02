from typing import List
import asyncio
import sys
import time
import traceback
from dataclasses import asdict
from math import pi
from typing import Optional, Union
from dtps import context

import argparse

from dt_node_utils import NodeType
from dt_node_utils.decorators import sidecar
from dt_node_utils.node import Node
from dtps.ergo_ui import DTPSContext
from dtps_http.object_queue import TransformError
from dtps_http.structures import RawData
from duckietown_messages.actuators.drone_control import DroneControl
from duckietown_messages.actuators.drone_mode import (
    DroneModeMsg,
    DroneModeResponse,
    Mode,
)
from duckietown_messages.actuators.drone_motor_command import DroneMotorCommand
# from duckietown_messages.actuators.attitude_pids_parameters import AttitudePIDParameters
from duckietown_messages.geometry_3d.quaternion import Quaternion
from duckietown_messages.sensors.angular_velocities import AngularVelocities
from duckietown_messages.sensors.battery import BatteryState
from duckietown_messages.sensors.imu import Imu
from duckietown_messages.sensors.linear_accelerations import LinearAccelerations
from duckietown_messages.standard.header import Header
from flight_controller_node.types import FlightControllerConfiguration, DroneMode, FCError

from mavsdk import System


DEG2RAD = pi / 180.0


def near_zero(n):
    """Set a number to zero if it is below a threshold value"""
    return 0 if abs(n) < 0.0001 else n


class FlightControllerNode(Node):
    """
    A class that implements the communication channels with the flight controller board via USB.
    It reads data from the IMU and the PWM signals going to the motors while relaying the command
    [R, P, Y, T] provided via ROS.

    """

    def __init__(self, config: str, arm_override = False):
        node_name: str = "flight_controller_node"
        super(FlightControllerNode, self).__init__(
            name=node_name, kind=NodeType.DRIVER, description="Flight controller driver"
        )

        self.configuration: FlightControllerConfiguration = (
            FlightControllerConfiguration.from_name(self.package, node_name, config)
        )

        # IMU reference frame id
        self._imu_frame_id: str = f"{self._robot_name}/imu_link"

        # internal state
        self._last_imu_msg = None
        self._last_published_mode: Optional[DroneMode] = None
        self._requested_mode: DroneMode = DroneMode.DISARMED
        self._current_mode: DroneMode = DroneMode.DISARMED

        # heartbeats
        self._heartbeat_joystick = time.perf_counter()
        self._heartbeat_pid = time.perf_counter()
        self._heartbeat_altitude = time.perf_counter()
        self._heartbeat_state_estimator = time.perf_counter()
        self._heartbeat_thr = 1.0  # Threshold for heartbeat checks [s]

        self._board = System()

        self._arm_override = arm_override

        # store the command to send to the flight controller, initialize as disarmed
        self._command = [ 1000, 1000, 1000, 1000,]
        self._last_command = [ 1000, 1000, 1000, 1000,]
    
    
    async def _switch_to_mode(self, mode: DroneMode, quiet: bool = False):
        """Update desired mode"""
        # switch mode
        # TODO: this is wrong, we can't wait for a new mode to come in to update _current_mode,
        #  it should be done according to the data coming from the flight controller
        self._current_mode = self._requested_mode
        self._requested_mode = mode
        
        await self._mode_to_function[mode]()

    async def _transform_set_mode(
        self,
        rd: RawData,
    ) -> Union[RawData, TransformError]:
        """Update desired mode"""
        drone_mode_msg : DroneModeMsg = DroneModeMsg.from_rawdata(rd) # type: ignore

        mode = DroneMode(drone_mode_msg.mode.value)
        do_switch = True
        # the user can only request to DISARM, ARM, and FLY
        if mode not in [DroneMode.DISARMED, DroneMode.ARMED, DroneMode.FLYING]:
            do_switch = False
        # make sure we are not violating a DISARM check
        if mode in [DroneMode.ARMED, DroneMode.FLYING] and self._should_disarm():
            mode = DroneMode.DISARMED
        # switch mode
        if do_switch:
            await self._switch_to_mode(mode)

        self.logger.info(f"Switched to mode {self._requested_mode}")

        await self.current_mode_queue.publish(
            DroneModeMsg(mode=Mode(self._requested_mode.value)).to_rawdata()
        )
        
        # respond
        return DroneModeResponse(
            previous_mode=Mode(self._current_mode.value),
            current_mode=Mode(self._requested_mode.value),
        ).to_rawdata()

    async def _transform_update_pids(self, rd: RawData) -> Union[RawData, TransformError]:
        """Update PID values"""
        return TransformError(400, "Failed to update PID values")
        try:
            msg: AttitudePIDParameters = AttitudePIDParameters.from_rawdata(
                rd
            )
            pids = AttitudePidGains.from_parameters_message(msg)
            pids_int = asdict(pids)
            # Cast all the values of the dictionary from float to int
            for key in pids_int:
                pids_int[key] = int(pids_int[key])
            return TransformError(400, "Failed to update PID values")

            if self._board.set_pids_rpy(**pids_int) is False:
                return TransformError(400, "Failed to update PID values")

            # respond
            return pids.to_parameters_message().to_rawdata()

        except Exception as e:
            return TransformError(
                400,
                f"Failed to update PID values, the following exception occurred: {e}",
            )

    async def _srv_zero_yaw_cb(self, _) -> Union[RawData, TransformError]:
        """Zero yaw"""

        try:
            self._board.calibration.calibrate_gyro()
        except Exception as e:
            return TransformError(400, str(e))

        # respond
        return RawData.cbor_from_native_object(
            {
                "success": True,
                "message": "Yaw zeroed",
            }
        )

    async def _srv_calibrate_imu_cb(self, _):
        """Calibrate IMU"""
        try:
            self._board.calibration.calibrate_level_horizon()
        except Exception as e:
            traceback.print_exc()
            return RawData.cbor_from_native_object(
                {"success": False, "message": str(e)}
            )
        # respond
        return RawData.cbor_from_native_object(
            {"success": True, "message": "IMU calibrated"}
        )


    @staticmethod
    def normalize_cmd(cmd):
        """
        Normalize the command values to the range [-1, 1].
        """
        return [
            float(cmd[0] / 1000 - 1.0),
            float(cmd[1] / 1000 - 1.0),
            float(cmd[2] / 1000 - 1.0),
            float(cmd[3] / 1000 - 1.0),
        ]
    
    @staticmethod
    def denormalize_cmd(cmd : List[float]) -> List[int]:
        """
        Denormalize the command values to the range [1000, 2000].
        """
        return [
            int(cmd[0] * 1000 + 1000),
            int(cmd[1] * 1000 + 1000),
            int(cmd[2] * 1000 + 1000),
            int(cmd[3] * 1000 + 1000),
        ]

    async def _flight_commands_cb(self, rd):
        msg: DroneControl = DroneControl.from_rawdata(rd)  # type: ignore
        """ Store and send the flight commands if the current mode is FLYING """
        if self._requested_mode is DroneMode.FLYING:
            # compile command to be sent to the flight controller board
            self._command = [msg.roll, msg.pitch, msg.throttle, msg.yaw]

    #@sidecar
    async def worker_parameters(self):
        await self.switchboard_ready.wait()
        # Create parameters queue
        self._parameters_queue = await (self.context / "parameters").queue_create( # type: ignore
            transform=self._transform_update_pids
        )
        await (self.switchboard / "flight_controller" / "parameters").expose( # type: ignore
            self._parameters_queue
        )

        # Expose parameters queue to the switchboard
        # await self._parameters_queue.publish(
        #         self._board.attitude_pid_gains.to_parameters_message().to_rawdata()
        # )

    async def worker(self):

        try:
            await self._board.connect(
                    system_address=f"serial://{self.configuration.serial.port}:{self.configuration.serial.baudrate}"
                )
        except Exception as e:
            raise e
        
        self._mode_to_function = {
            DroneMode.DISARMED: self._board.action.disarm,
            DroneMode.ARMED: self._board.action.arm,
            DroneMode.FLYING: self._board.action.arm,
        }
        
        await self.dtps_init(self.configuration)
        # Create queues OUT
        executed_commands_queue: DTPSContext = await (
            self.context / "out" / "commands" / "executed" # type: ignore
        ).queue_create()
        self.current_mode_queue: DTPSContext = await (
            self.context / "out" / "mode" / "current" # type: ignore
        ).queue_create()

        # Expose queues to the switchboard
        await (self.switchboard / "flight_controller" / "mode" / "current").expose( # type: ignore
            self.current_mode_queue
        )
        await (self.switchboard / "flight_controller" / "commands" / "executed").expose( # type: ignore
            executed_commands_queue
        )

        # Create queues IN
        heartbeat_altitude: DTPSContext = await (
            self.context / "in" / "heartbeat" / "altitude" # type: ignore
        ).queue_create()
        heartbeat_joystick: DTPSContext = await (
            self.context / "in" / "heartbeat" / "joystick" # type: ignore
        ).queue_create()
        heartbeat_pid: DTPSContext = await (
            self.context / "in" / "heartbeat" / "pid" # type: ignore
        ).queue_create()
        heartbeat_state_estimator: DTPSContext = await (
            self.context / "in" / "heartbeat" / "state_estimator" # type: ignore
        ).queue_create()
        commands_queue: DTPSContext = await (
            self.context / "in" / "commands" # type: ignore
        ).queue_create()

        # Subscribe heartbeat callbacks
        await heartbeat_altitude.subscribe(self._heartbeat_altitude_cb)
        await heartbeat_joystick.subscribe(self._heartbeat_joystick_cb)
        await heartbeat_pid.subscribe(self._heartbeat_pid_cb)
        await heartbeat_state_estimator.subscribe(self._heartbeat_state_estimator_cb)

        # Subscribe to commands queue
        await commands_queue.subscribe(self._flight_commands_cb)

        # Create services queues and add transforms
        # WORKAROUND: until switchboard is fixed, we need to create the queues in the switchboard
        rpc_call_context = await context("rpc", environment={"DTPS_BASE_RPC":"create:http://localhost:2120/"})

        zero_yaw_queue = await (
            rpc_call_context / "imu" / "zero_yaw"
        ).queue_create(
            transform=self._srv_zero_yaw_cb
        )
        calibrate_imu_queue = await (
            rpc_call_context / "imu" / "calibrate"
        ).queue_create(
            transform=self._srv_calibrate_imu_cb
        )
        set_mode_queue = await (rpc_call_context / "flight_controller" / "mode" / "set").queue_create(
            transform=self._transform_set_mode
        )

        # Expose queues to the switchboard
        await (self.switchboard / "flight_controller" / "commands").expose( # type: ignore
            commands_queue
        )

        # WORKAROUND END

        # Expose heartbeat queues to the switchboard
        await (self.switchboard / "heartbeat" / "altitude").expose(heartbeat_altitude) # type: ignore
        await (self.switchboard / "heartbeat" / "joystick").expose(heartbeat_joystick) # type: ignore
        await (self.switchboard / "heartbeat" / "pid").expose(heartbeat_pid) # type: ignore
        await (self.switchboard / "heartbeat" / "state_estimator").expose( # type: ignore
            heartbeat_state_estimator
        )

        # Expose node to the switchboard
        await self.dtps_expose()

        dt: float = 1.0 / self.configuration.frequency.commands

        # Set the board to be controlled offboard and in stabilized mode
        await self._send_flight_commands(executed_commands_queue)

        await self._board.manual_control.start_altitude_control()
        
        self.logger.info("Starting main control loop")
        async with executed_commands_queue.publisher_context() as cmd_pub:
            main_loop_start_time = time.perf_counter()
            try:
                while not self.is_shutdown and self._event_loop is not None:
                    profiling_start_time = time.perf_counter()
                    loop_start_time = self._event_loop.time()
                    
                    if self._arm_override:
                        await self.perform_arm_override(main_loop_start_time)

                    # if the current mode is anything other than disarmed, preform as safety check
                    if self._requested_mode is not DroneMode.DISARMED:
                        # break the loop if a safety check has failed
                        if self._should_disarm():
                            self.logger.info("Should disarm.")
                            
                            await self._board.action.disarm()
                            
                            # sleep for the remainder of the loop time
                            cycle_time = self._event_loop.time() - loop_start_time
                            await asyncio.sleep(dt - cycle_time)
                            continue

                    # noinspection PyBroadException
                    try:
                        # update and send the flight commands to the board
                        await self._compute_flight_commands()
                        await self._send_flight_commands(cmd_pub) # type: ignore

                        # publish the current mode
                        if self._last_published_mode != self._requested_mode:
                            # TODO: do we need to publish the mode again here Do we need it to be published continuously?
                            # await self.current_mode_queue.publish(
                            #     DroneModeMsg(mode=self._requested_mode.value).to_rawdata()
                            # )
                            self._last_published_mode = self._requested_mode

                    except FCError:
                        self.logwarn(
                            "Could not talk to the flight controller" + str(FCError)
                        )
                        continue

                    # sleep for the remainder of the loop time
                    cycle_time = self._event_loop.time() - loop_start_time
                    
                    await asyncio.sleep(max(0,dt-cycle_time))
                    self.logdebug(f"CMD frequency: {1/(time.perf_counter()-profiling_start_time)} Hz")

                await self._board.action.disarm()

            except Exception:
                traceback.print_exc()

        self.loginfo("Shutdown received, disarming...")
        await self._board.action.disarm()
        time.sleep(0.5)

    async def perform_arm_override(self, main_loop_start_time):
        now = time.perf_counter()

        if now - main_loop_start_time > 4:
            await self._switch_to_mode(DroneMode.FLYING, quiet=True)
            self.logger.info("Arm override enabled, flying the drone.")
        
        if 4 > now - main_loop_start_time > 2:
            await self._switch_to_mode(DroneMode.ARMED, quiet=True)
            self.logger.info("Arm override enabled, arming the drone.")



    @sidecar
    async def worker_battery(self):
        if self._event_loop is None:
            return

        await self.switchboard_ready.wait()

        battery_queue = await (self.context / "out" / "battery").queue_create() # type: ignore
        await (self.switchboard / "sensor" / "battery").expose(battery_queue) # type: ignore

        while not self.is_shutdown:    
            async for battery in self._board.telemetry.battery():
                battery_msg = BatteryState(
                    voltage=battery.voltage_v,
                    present=True
                    if battery.voltage_v > 6.0
                    else False,  # ~5V: power from Pi | 7V to 12.6V: power from battery
                )
                await battery_queue.publish(battery_msg.to_rawdata())

    @sidecar
    async def worker_motor_pwm(self):
        await self.switchboard_ready.wait()

        if self._event_loop is None:
            return

        motors_queue = await (self.context / "out" / "motors").queue_create() # type: ignore
        await (self.switchboard / "actuator" / "motors").expose(motors_queue) # type: ignore

        while not self.is_shutdown:
            async for m in self._board.telemetry.actuator_output_status():
                try:
                    # read PWM signals going to the motors
                    motor_msg = DroneMotorCommand(
                        minimum=self.configuration.motor_command_range[0],
                        maximum=self.configuration.motor_command_range[1],
                        m1=m[0],
                        m2=m[1],
                        m3=m[2],
                        m4=m[3],
                    )
                    await motors_queue.publish(motor_msg.to_rawdata())
                except Exception:
                    traceback.print_exc()

    @sidecar
    async def worker_imu(self):
        await self.switchboard_ready.wait()

        data_queue = await (self.context / "out" / "data").queue_create() # type: ignore
        await (self.switchboard / "sensor" / "imu" / "data").expose(data_queue) # type: ignore

        # TODO: check if this returns the acceleration in body frame!!!
        imu_iter = self._board.telemetry.imu()
        attitude_quaternion_iter = self._board.telemetry.attitude_quaternion()
        ang_vel_iter = self._board.telemetry.attitude_angular_velocity_body()

        async with data_queue.publisher_context() as data_pub:
            while not self.is_shutdown:
                try:
                    imu = await imu_iter.__anext__()
                    
                    # process acceleration data
                    a_x, a_y, a_z = imu.acceleration_frd.forward_m_s2, imu.acceleration_frd.right_m_s2, imu.acceleration_frd.down_m_s2
                    acceleration_message = LinearAccelerations(
                        x=near_zero(a_x),
                        y=near_zero(a_y),
                        z=near_zero(a_z),
                    )
                    
                    # process angular velocity data
                    ang_vel = await ang_vel_iter.__anext__()

                    angular_velocity_message = AngularVelocities(
                        x=ang_vel.roll_rad_s, y=ang_vel.pitch_rad_s, z=ang_vel.yaw_rad_s
                    )

                    # process attitude data
                    q = await attitude_quaternion_iter.__anext__()
                    orientation_msg = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)

                except Exception as e:
                    traceback.print_exc()
                    self.logwarn(f"IMU Comm Loss: {e}")
                else:
                    # pack Imu data
                    imu_message = Imu(
                        header=Header(frame=self._imu_frame_id),
                        angular_velocity=angular_velocity_message,
                        linear_acceleration=acceleration_message,
                        orientation=orientation_msg,
                    ) # type: ignore
                    await data_pub.publish(imu_message.to_rawdata())


    async def _compute_flight_commands(self):
        """Set command values if the mode is ARMED or DISARMED"""
        if self._requested_mode is DroneMode.DISARMED:
            # disarm
            await self._switch_to_mode(DroneMode.DISARMED, quiet=True)

        elif self._requested_mode is DroneMode.ARMED:
            # arm
            if self._current_mode is DroneMode.DISARMED:
                # not yet armed
                await self._switch_to_mode(DroneMode.ARMED, quiet=True)

            elif self._current_mode is DroneMode.ARMED:
                # already armed
                pass

        elif self._requested_mode is DroneMode.FLYING:
            # flying
            await self._switch_to_mode(DroneMode.FLYING, quiet=True)

    async def _send_flight_commands(self, queue: DTPSContext):
        """Send commands to the flight controller board"""
        try:
            self.logdebug(f"Sending command to board: {self._command}")

            roll, pitch, throttle, yaw = self.normalize_cmd(self._command)
            await self._board.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)

            # keep track of the last command sent
            if self._command != self._last_command:
                self._last_command = self._command.copy()

            await queue.publish(
                DroneControl(
                    roll=self._command[0],
                    pitch=self._command[1],
                    yaw=self._command[2],
                    throttle=self._command[3],
                ).to_rawdata()
            )
        except Exception as e:
            self.logerr(f"Error communicating with board {e}")

    # heartbeat callbacks: These update the last time that data was received from a node

    async def _heartbeat_joystick_cb(self, _):
        """Update joystick heartbeat"""
        self._heartbeat_joystick = time.perf_counter()

    async def _heartbeat_pid_cb(self, _):
        """Update pid_controller heartbeat"""
        self._heartbeat_pid = time.perf_counter()

    async def _heartbeat_altitude_cb(self, _):
        """Update altitude sensor heartbeat"""
        self._heartbeat_altitude = time.perf_counter()

    async def _heartbeat_state_estimator_cb(self, _):
        """Update state_estimator heartbeat"""
        self._heartbeat_state_estimator = time.perf_counter()

    def _should_disarm(self):
        """
        Disarm the drone if the battery values are too low or if there is a
        missing heartbeat
        """
        curr_time = time.perf_counter()
        disarm = False

        # - joystick
        if (
            self.configuration.heartbeats.joystick
            and curr_time - self._heartbeat_joystick > self._heartbeat_thr
        ):
            self.logfatal(
                "\nSafety Failure: not receiving flight commands. "
                "Check the 'Joystick' node\n"
            )
            disarm = True
        # - pid
        if (
            self.configuration.heartbeats.pid
            and curr_time - self._heartbeat_pid > self._heartbeat_thr
        ):
            self.logfatal(
                "\nSafety Failure: not receiving flight commands. "
                "Check the 'PID' node\n"
            )
            disarm = True
        # - altitude
        if (
            self.configuration.heartbeats.altitude
            and curr_time - self._heartbeat_altitude > self._heartbeat_thr
        ):
            self.logfatal(
                "\nSafety Failure: not receiving data from the IR sensor. "
                "Check the 'altitude' node\n"
            )
            disarm = True
        # - state_estimator
        if (
            self.configuration.heartbeats.state_estimator
            and curr_time - self._heartbeat_state_estimator > self._heartbeat_thr
        ):
            self.logfatal(
                "\nSafety Failure: not receiving a state estimate. "
                "Check the 'state estimator' node\n"
            )
            disarm = True

        return disarm

    def on_shutdown(self):
        """
        Disarm the drone and quits the flight controller node.
        """
        self.loginfo("Shutting down the flight controller node")
 
        asyncio.run(self._switch_to_mode(DroneMode.DISARMED))
        sys.exit()


def main():

    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=str, required=True, help="Name of the configuration"
    )

    parser.add_argument(
        "--arm-override", action="store_true", help="Override the arm check", default=False
    )

    args: argparse.Namespace = parser.parse_args()

    # create node
    node: FlightControllerNode = FlightControllerNode(config=args.config, arm_override=args.arm_override)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
