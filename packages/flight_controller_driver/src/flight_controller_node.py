#!/usr/bin/env python3

import traceback
from enum import IntEnum
from threading import Semaphore
from typing import List, Optional

import numpy as np
import rospy
import sys
import tf
import time
from duckietown_msgs.msg import DroneMode as DroneModeMsg, DroneControl, DroneMotorCommand

from sensor_msgs.msg import Imu, BatteryState
from serial import SerialException
from std_msgs.msg import Header, Empty

from duckietown_msgs.srv import SetDroneMode, SetDroneModeResponse
from std_srvs.srv import Trigger, TriggerResponse

from dt_class_utils import DTReminder
from duckietown.dtros import DTROS, NodeType
from h2r_multiwii import MultiWii


class DroneMode(IntEnum):
    DISARMED = 0
    ARMED = 1
    FLYING = 2
    IDLE = 3


class FCError(SerialException):
    pass


def near_zero(n):
    """ Set a number to zero if it is below a threshold value """
    return 0 if abs(n) < 0.0001 else n


class FlightController(DTROS):
    """
    A class that implements the communication channels with the flight controller board via USB.
    It reads data from the IMU and the PWM signals going to the motors while relaying the command
    [R, P, Y, T] provided via ROS.

    """

    def __init__(self):
        super(FlightController, self).__init__(node_name="flight_controller_node",
                                               node_type=NodeType.DRIVER)
        # parameters
        self._veh = rospy.get_param("~veh")
        self._device = rospy.get_param("~device")
        self._frequency = rospy.get_param("~frequency")
        self._heartbeats = rospy.get_param("~heartbeats")
        self._rc_commands = rospy.get_param("~rc_commands")
        self._accelerometer_calib = rospy.get_param("~accelerometer")
        self._motor_command_range = rospy.get_param("~motor_command_range")

        # mode -> RC command mapping
        self._mode_to_rc_command = {
            DroneMode.DISARMED: self._rc_commands["disarm"],
            DroneMode.ARMED: self._rc_commands["arm"],
            DroneMode.IDLE: self._rc_commands["idle"],
            DroneMode.FLYING: None,
        }

        # number of commands to publish to FC to arm/disarm, hold stick for at least 1.0 secs
        self._mode_change_cycles = 1.0 * self._frequency["commands"]
        self._mode_change_counter = 0

        # internal state
        self._last_imu_msg = None
        self._clock = rospy.Rate(self._frequency["commands"])
        self._lock = Semaphore(1)
        self._last_published_mode: Optional[DroneMode] = None
        self._requested_mode: DroneMode = DroneMode.DISARMED
        self._current_mode: DroneMode = DroneMode.DISARMED

        # heartbeats
        self._heartbeat_joystick = rospy.Time()
        self._heartbeat_pid = rospy.Time()
        self._heartbeat_altitude = rospy.Time()
        self._heartbeat_state_estimator = rospy.Time()
        self._heartbeat_thr = rospy.Duration.from_sec(1)

        # (try to) connect to the flight controller board
        self._board: Optional[MultiWii] = None
        self._open_board()
        if self._board is None:
            return

        # reminders
        self._motors_reminder = DTReminder(frequency=self._frequency["motors"])
        self._imu_reminder = DTReminder(frequency=self._frequency["imu"])

        # publishers
        self._imu_pub = rospy.Publisher("~imu", Imu, queue_size=1)
        self._motor_pub = rospy.Publisher("~motors", DroneMotorCommand, queue_size=1)
        self._bat_pub = rospy.Publisher("~battery", BatteryState, queue_size=1)
        self._mode_pub = rospy.Publisher("~mode/current", DroneModeMsg, queue_size=1, latch=True)
        self._commands_pub = rospy.Publisher('~commands/executed', DroneControl, queue_size=1)

        # subscribers
        rospy.Subscriber('~commands', DroneControl, self._fly_commands_cb, queue_size=1)

        # services
        self._srv_set_mode = rospy.Service("~set_mode", SetDroneMode, self._srv_set_mode_cb)
        self._srv_calib_imu = rospy.Service("~calibrate_imu", Trigger, self._srv_calibrate_imu_cb)

        # heartbeats
        rospy.Subscriber("~heartbeat/altitude", Empty, self._heartbeat_altitude_cb, queue_size=1)
        rospy.Subscriber("~heartbeat/joystick", Empty, self._heartbeat_joystick_cb, queue_size=1)
        rospy.Subscriber("~heartbeat/pid", Empty, self._heartbeat_pid_cb, queue_size=1)
        rospy.Subscriber("~heartbeat/state_estimator", Empty, self._heartbeat_state_estimator_cb, queue_size=1)

        # store the command to send to the flight controller, initialize as disarmed
        self._command = self.rc_command(DroneMode.DISARMED)
        self._last_command = self.rc_command(DroneMode.DISARMED)

        # accelerometer parameters
        self.accRawToMss = 9.8 / self._accelerometer_calib["z"]
        self.accZeroX = self._accelerometer_calib["x"] * self.accRawToMss
        self.accZeroY = self._accelerometer_calib["y"] * self.accRawToMss
        self.accZeroZ = self._accelerometer_calib["z"] * self.accRawToMss

    def rc_command(self, mode: DroneMode) -> Optional[List[int]]:
        """
        Provides the raw RC commands for a given mode.

        Args:
            mode: Mode to retrieve the raw RC command for.

        Returns:
            Raw RC command to send to the flight controller to trigger the given mode.

        """
        return self._mode_to_rc_command.get(mode)

    def _needs_heartbeat(self, name: str) -> bool:
        return self._heartbeats.get(name, False) is True

    def _switch_to_mode(self, mode: DroneMode, quiet: bool = False):
        """ Update desired mode """
        # switch mode
        # TODO: this is wrong, we can't wait for a new mode to come in to update _current_mode,
        #  it should be done according to the data coming from the flight controller
        self._current_mode = self._requested_mode
        self._requested_mode = mode
        if not quiet:
            self._compute_flight_commands()

    def _srv_set_mode_cb(self, req):
        """ Update desired mode """
        mode = DroneMode(req.mode.mode)
        do_switch = True
        # the user can only request to DISARM, ARM, and FLY
        if mode not in [DroneMode.DISARMED, DroneMode.ARMED, DroneMode.FLYING]:
            do_switch = False
        # make sure we are not violating a DISARM check
        if mode in [DroneMode.ARMED, DroneMode.FLYING] and self._should_disarm():
            mode = DroneMode.DISARMED
        # switch mode
        if do_switch:
            self._switch_to_mode(mode)
        self._mode_pub.publish(self._requested_mode)
        # respond
        return SetDroneModeResponse(
            previous_mode=DroneModeMsg(mode=self._current_mode.value),
            current_mode=DroneModeMsg(mode=self._requested_mode.value),
        )

    def _srv_calibrate_imu_cb(self, _):
        """ Calibrate IMU """
        self.loginfo("Calibrating IMU...")
        try:
            with self._lock:
                self._board.sendCMD(0, MultiWii.ACC_CALIBRATION, [])
                self._board.receiveDataPacket()
        except Exception as e:
            traceback.print_exc()
            return TriggerResponse(success=False, message=str(e))
        # respond
        return TriggerResponse(success=True)

    def _fly_commands_cb(self, msg):
        """ Store and send the flight commands if the current mode is FLYING """
        if self._requested_mode is DroneMode.FLYING:
            # compile command to be sent to the flight controller board via MultiWii
            self._command = [int(msg.roll), int(msg.pitch), int(msg.yaw), int(msg.throttle)]

    def run(self):
        # noinspection PyBroadException
        try:
            while not self.is_shutdown:
                # if the current mode is anything other than disarmed, preform as safety check
                if self._requested_mode is not DroneMode.DISARMED:
                    # break the loop if a safety check has failed
                    if self._should_disarm():
                        self.loginfo("Should disarm.")
                        self._switch_to_mode(DroneMode.DISARMED)
                        # sleep for the remainder of the loop time
                        self._clock.sleep()
                        continue

                with self._lock:
                    # noinspection PyBroadException
                    try:
                        # update and publish flight controller readings
                        if self._imu_reminder.is_time():
                            imu_msg = self._read_imu_message()
                            self._imu_pub.publish(imu_msg)

                        # read PWM signals going to the motors
                        if self._motors_reminder.is_time():
                            motor_msg = self._read_motor_pwm_signals()
                            self._motor_pub.publish(motor_msg)

                        # update and send the flight commands to the board
                        self._compute_flight_commands()
                        self._send_flight_commands()

                        # publish the current mode
                        if self._last_published_mode != self._requested_mode:
                            self._mode_pub.publish(self._requested_mode)
                            self._last_published_mode = self._requested_mode
                    except FCError:
                        self.logwarn("Could not talk to the flight controller")
                        continue

                # sleep for the remainder of the loop time
                self._clock.sleep()
        except Exception:
            traceback.print_exc()
        finally:
            self.loginfo('Shutdown received, disarming...')
            self._board.sendCMD(8, MultiWii.SET_RAW_RC, self.rc_command(DroneMode.DISARMED))
            self._board.receiveDataPacket()
            time.sleep(0.5)

    def _compute_flight_commands(self):
        """ Set command values if the mode is ARMED or DISARMED """
        if self._requested_mode is DroneMode.DISARMED:
            # disarm
            self._command = self.rc_command(DroneMode.DISARMED)
            self._switch_to_mode(DroneMode.DISARMED, quiet=True)

        elif self._requested_mode is DroneMode.ARMED:
            # arm
            if self._current_mode is DroneMode.DISARMED:
                # not yet armed
                self._command = self.rc_command(DroneMode.ARMED)
                self._switch_to_mode(DroneMode.ARMED, quiet=True)

            elif self._current_mode is DroneMode.ARMED:
                # already armed
                if self._mode_change_counter < self._mode_change_cycles:
                    self._command = self.rc_command(DroneMode.ARMED)
                    self._mode_change_counter += 1
                else:
                    self._command = self.rc_command(DroneMode.IDLE)

        if self._requested_mode is not DroneMode.ARMED:
            self._mode_change_counter = 0

    def _send_flight_commands(self):
        """ Send commands to the flight controller board """
        try:
            self._board.sendCMD(8, MultiWii.SET_RAW_RC, self._command)
            self._board.receiveDataPacket()
            # keep track of the last command sent
            if self._command != self._last_command:
                self._last_command = self._command
            self._commands_pub.publish(DroneControl(*self._command))
        except Exception as e:
            self.logerr(f"Error communicating with board {e}")

    # heartbeat callbacks: These update the last time that data was received from a node

    def _heartbeat_joystick_cb(self, _):
        """ Update joystick heartbeat """
        self._heartbeat_joystick = rospy.Time.now()

    def _heartbeat_pid_cb(self, _):
        """ Update pid_controller heartbeat """
        self._heartbeat_pid = rospy.Time.now()

    def _heartbeat_altitude_cb(self, _):
        """ Update altitude sensor heartbeat """
        self._heartbeat_altitude = rospy.Time.now()

    def _heartbeat_state_estimator_cb(self, _):
        """ Update state_estimator heartbeat """
        self._heartbeat_state_estimator = rospy.Time.now()

    def _should_disarm(self):
        """
        Disarm the drone if the battery values are too low or if there is a
        missing heartbeat
        """
        curr_time = rospy.Time.now()
        disarm = False

        # - joystick
        if self._needs_heartbeat("joystick") and \
                curr_time - self._heartbeat_joystick > self._heartbeat_thr:
            self.logfatal("\nSafety Failure: not receiving flight commands. "
                          "Check the 'Joystick' node\n")
            disarm = True
        # - pid
        if self._needs_heartbeat("pid") and \
                curr_time - self._heartbeat_pid > self._heartbeat_thr:
            self.logfatal("\nSafety Failure: not receiving flight commands. "
                          "Check the 'PID' node\n")
            disarm = True
        # - altitude
        if self._needs_heartbeat("altitude") and \
                curr_time - self._heartbeat_altitude > self._heartbeat_thr:
            self.logfatal("\nSafety Failure: not receiving data from the IR sensor. "
                          "Check the 'altitude' node\n")
            disarm = True
        # - state_estimator
        if self._needs_heartbeat("state_estimator") and \
                curr_time - self._heartbeat_state_estimator > self._heartbeat_thr:
            self.logfatal("\nSafety Failure: not receiving a state estimate. "
                          "Check the 'state estimator' node\n")
            disarm = True

        return disarm

    def _read_motor_pwm_signals(self):
        """
        Reads the motor signals sent by the flight controller to the ESCs.
        """
        try:
            # read m1, m2, m3, m4
            self._board.getData(MultiWii.MOTOR)
        except Exception as e:
            self.logwarn(f"Unable to get MOTOR data {e}, retry...")
            raise FCError(f"Unable to get MOTOR data {e}, retry...")

        # create Motor message
        return DroneMotorCommand(
            header=Header(stamp=rospy.Time.now()),
            minimum=self._motor_command_range[0],
            maximum=self._motor_command_range[1],
            m1=int(self._board.motor["m1"]),
            m2=int(self._board.motor["m2"]),
            m3=int(self._board.motor["m3"]),
            m4=int(self._board.motor["m4"]),
        )

    def _read_imu_message(self):
        """
        Compute the ROS IMU message by reading data from the board.
        """
        try:
            # read roll, pitch, heading
            self._board.getData(MultiWii.ATTITUDE)
            # read lin_acc_x, lin_acc_y, lin_acc_z
            self._board.getData(MultiWii.RAW_IMU)
        except Exception as e:
            self.logwarn(f"Unable to get IMU data {e}, retry...")
            raise FCError(f"Unable to get IMU data {e}, retry...")

        # create empty message
        msg = Imu()
        msg.header = Header(
            frame_id=f"/{self._veh}/imu",
            stamp=rospy.Time.now()
        )

        # calculate values to update imu_message:
        roll = np.deg2rad(self._board.attitude['angx'])
        pitch = -np.deg2rad(self._board.attitude['angy'])
        heading = np.deg2rad(self._board.attitude['heading'])
        # Note that at pitch angles near 90 degrees, the roll angle reading can fluctuate a lot
        # transform heading (similar to yaw) to standard math conventions, which
        # means angles are in radians and positive rotation is CCW
        heading = (-heading) % (2 * np.pi)

        # transform euler angles into quaternion
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, heading)
        # calculate the linear accelerations
        lin_acc_x = self._board.rawIMU['ax'] * self.accRawToMss - self.accZeroX
        lin_acc_y = self._board.rawIMU['ay'] * self.accRawToMss - self.accZeroY
        lin_acc_z = self._board.rawIMU['az'] * self.accRawToMss - self.accZeroZ

        # Rotate the IMU frame to align with our convention for the drone's body
        # frame. IMU: x is forward, y is left, z is up. We want: x is right,
        # y is forward, z is up.
        lin_acc_x_drone_body = -lin_acc_y
        lin_acc_y_drone_body = lin_acc_x
        lin_acc_z_drone_body = lin_acc_z

        # Account for gravity's affect on linear acceleration values when roll
        # and pitch are nonzero. When the drone is pitched at 90 degrees, for
        # example, the z acceleration reads out as -9.8 m/s^2. This makes sense,
        # as the IMU, when powered up / when the calibration script is called,
        # zeros the body-frame z-axis acceleration to 0, but when it's pitched
        # 90 degrees, the body-frame z-axis is perpendicular to the force of
        # gravity, so, as if the drone were in free-fall (which was roughly
        # confirmed experimentally), the IMU reads -9.8 m/s^2 along the z-axis.
        g = 9.8
        lin_acc_x_drone_body = lin_acc_x_drone_body + g * np.sin(roll) * np.cos(pitch)
        lin_acc_y_drone_body = lin_acc_y_drone_body + g * np.cos(roll) * (-np.sin(pitch))
        lin_acc_z_drone_body = lin_acc_z_drone_body + g * (1 - np.cos(roll) * np.cos(pitch))

        # angular velocities
        angvx = 0.0
        angvy = 0.0
        angvz = 0.0

        if self._last_imu_msg is not None:
            # calculate the angular velocities of roll, pitch, and yaw in rad/s
            # When first powered up, heading should read near 0, get the previous R, P, Y values
            previous_quat = self._last_imu_msg.orientation
            previous_t = self._last_imu_msg.header.stamp
            quat = [previous_quat.x, previous_quat.y, previous_quat.z, previous_quat.w]
            previous_roll, previous_pitch, previous_heading = \
                tf.transformations.euler_from_quaternion(quat)

            # Although quaternion_from_euler takes a heading in range [0, 2pi),
            # euler_from_quaternion returns a heading in range [0, pi] or [0, -pi).
            # Thus need to convert the returned heading back into the range [0, 2pi).
            previous_heading = previous_heading % (2 * np.pi)

            current_t = msg.header.stamp
            dt = current_t.to_sec() - previous_t.to_sec()
            dr = roll - previous_roll
            dp = pitch - previous_pitch
            dh = heading - previous_heading
            angvx = near_zero(dr / dt)
            angvy = near_zero(dp / dt)
            angvz = near_zero(dh / dt)

        # populate the imu message:
        # - orientation
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        # - angular velocities
        msg.angular_velocity.x = angvx
        msg.angular_velocity.y = angvy
        msg.angular_velocity.z = angvz
        # - linear accelerations
        msg.linear_acceleration.x = lin_acc_x_drone_body
        msg.linear_acceleration.y = lin_acc_y_drone_body
        msg.linear_acceleration.z = lin_acc_z_drone_body
        # store last IMU message so that we can compute angular velocities by differentiating
        self._last_imu_msg = msg
        # ---
        return msg

    def _open_board(self):
        """ Connect to the flight controller board """
        try:
            board = MultiWii(self._device)
        except SerialException:
            self.logfatal(f"Cannot connect to the flight controller board at '{self._device}'. "
                          f"The USB is unplugged. Please check connection.")
            sys.exit()
        # make sure we have a connection to the board
        if board is None:
            self.logfatal(f"The flight controller board could not be found at '{self._device}'")
            sys.exit()
        # ---
        self._board = board

    def on_shutdown(self):
        """
        Disarm the drone and quits the flight controller node.
        """
        if self._board is None:
            return
        self.loginfo("Disarming!")
        self._board.sendCMD(8, MultiWii.SET_RAW_RC, self.rc_command(DroneMode.DISARMED))
        self._board.receiveDataPacket()
        self._mode_pub.publish(DroneModeMsg(mode=DroneMode.DISARMED.value))
        rospy.sleep(0.5)
        sys.exit()


def main():
    # run flight controller driver communication loop
    fc = FlightController()
    fc.run()


if __name__ == '__main__':
    main()
