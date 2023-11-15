#!/usr/bin/env python3
import copy
import json
import os
import traceback
from enum import IntEnum
from threading import Semaphore
from typing import List, Optional

from collections import defaultdict
from dataclasses import dataclass

import numpy as np
import rospy
import sys
import tf
import time
import yaml
from duckietown_msgs.msg import DroneMode as DroneModeMsg, DroneControl, DroneMotorCommand
from duckietown_msgs.srv import SetDroneMode, SetDroneModeResponse
from sensor_msgs.msg import Imu, BatteryState
from serial import SerialException
from serial.tools.list_ports import grep as serial_grep
from std_msgs.msg import Header, Empty
from std_srvs.srv import Trigger, TriggerResponse

from dt_class_utils import DTReminder
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from yamspy import MSPy


@dataclass
class MultiWiiRpyPid:
    roll_p: int
    roll_i: int
    roll_d: int

    pitch_p: int
    pitch_i: int
    pitch_d: int

    yaw_p: int
    yaw_i: int
    yaw_d: int

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
        self._rc_commands = rospy.get_param("~rc_commands")
        self._motor_command_range = rospy.get_param("~motor_command_range")
        self._heartbeats = DTParam("~heartbeats", param_type=ParamType.DICT)

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
        self._board: Optional[MSPy] = None
        self._open_board()
        if self._board is None:
            return

        # TODO: if update PID param failed with FC, ros param and FC PIDs are inconsistent
        # low priority. the params will be of the true values on next container start-up

        # obtain default PID values
        self.pids = defaultdict(dict)
        initial_rpy_pids: MultiWiiRpyPid = self._get_pid_values_board()
        self._param_roll_P = DTParam('~roll_P', default=initial_rpy_pids.roll_p,
                                     param_type=ParamType.INT)
        self._param_roll_P.register_update_callback(
            lambda: self._board.set_pids_rpy(roll_p=self._param_roll_P.value))
        self._param_roll_I = DTParam('~roll_I', default=initial_rpy_pids.roll_i,
                                     param_type=ParamType.INT)
        self._param_roll_I.register_update_callback(
            lambda: self._board.set_pids_rpy(roll_i=self._param_roll_I.value))
        self._param_roll_D = DTParam('~roll_D', default=initial_rpy_pids.roll_d,
                                     param_type=ParamType.INT)
        self._param_roll_D.register_update_callback(
            lambda: self._board.set_pids_rpy(roll_d=self._param_roll_D.value))

        self._param_pitch_P = DTParam('~pitch_P', default=initial_rpy_pids.pitch_p,
                                      param_type=ParamType.INT)
        self._param_pitch_P.register_update_callback(
            lambda: self._board.set_pids_rpy(pitch_p=self._param_pitch_P.value))
        self._param_pitch_I = DTParam('~pitch_I', default=initial_rpy_pids.pitch_i,
                                      param_type=ParamType.INT)
        self._param_pitch_I.register_update_callback(
            lambda: self._board.set_pids_rpy(pitch_i=self._param_pitch_I.value))
        self._param_pitch_D = DTParam('~pitch_D', default=initial_rpy_pids.pitch_d,
                                      param_type=ParamType.INT)
        self._param_pitch_D.register_update_callback(
            lambda: self._board.set_pids_rpy(pitch_d=self._param_pitch_D.value))

        self._param_yaw_P = DTParam('~yaw_P', default=initial_rpy_pids.yaw_p,
                                    param_type=ParamType.INT)
        self._param_yaw_P.register_update_callback(
            lambda: self._board.set_pids_rpy(yaw_p=self._param_yaw_P.value))
        self._param_yaw_I = DTParam('~yaw_I', default=initial_rpy_pids.yaw_i,
                                    param_type=ParamType.INT)
        self._param_yaw_I.register_update_callback(
            lambda: self._board.set_pids_rpy(yaw_i=self._param_yaw_I.value))
        self._param_yaw_D = DTParam('~yaw_D', default=initial_rpy_pids.yaw_d,
                                    param_type=ParamType.INT)
        self._param_yaw_D.register_update_callback(
            lambda: self._board.set_pids_rpy(yaw_d=self._param_yaw_D.value))

        # reminders
        self._motors_reminder = DTReminder(frequency=self._frequency["motors"])
        self._imu_reminder = DTReminder(frequency=self._frequency["imu"])
        self._battery_reminder = DTReminder(frequency=self._frequency["battery"])

        # store the command to send to the flight controller, initialize as disarmed
        self._command = self.rc_command(DroneMode.DISARMED)
        self._last_command = self.rc_command(DroneMode.DISARMED)

        # accelerometer calibration
        calibs_folder = '/data/config/calibrations/accelerometer'
        os.makedirs(calibs_folder, exist_ok=True)
        self._accelerometer_frame_id = os.path.join(rospy.get_namespace(), 'accelerometer')
        self._calib_file = os.path.join(calibs_folder, f"{self._veh}.yaml")
        self._default_accelerometer_calib = {
            "x": 0,
            "y": 0,
            "z": 512
        }

        # locate calibration yaml file or use the default otherwise
        if not os.path.isfile(self._calib_file):
            self.logwarn(f"Calibration not found: {self._calib_file}.\n Using default instead.")
            self._accelerometer_calib = copy.deepcopy(self._default_accelerometer_calib)
        else:
            with open(self._calib_file, "rt") as fin:
                self._accelerometer_calib = yaml.safe_load(fin)
        self.loginfo(f"Using accelerometer calibration: {self._accelerometer_calib}")

        # accelerometer parameters
        self._apply_calibration()

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
        rospy.Subscriber("~heartbeat/state_estimator", Empty, self._heartbeat_state_estimator_cb,
                         queue_size=1)

    def rc_command(self, mode: DroneMode) -> Optional[List[int]]:
        """
        Provides the raw RC commands for a given mode.

        Args:
            mode: Mode to retrieve the raw RC command for.

        Returns:
            Raw RC command to send to the flight controller to trigger the given mode.

        """
        return self._mode_to_rc_command.get(mode)

    def _apply_calibration(self):
        self.accRawToMss = 9.81 / self._accelerometer_calib["z"]
        self.accZeroX = self._accelerometer_calib["x"] * self.accRawToMss
        self.accZeroY = self._accelerometer_calib["y"] * self.accRawToMss
        self.accZeroZ = self._accelerometer_calib["z"] * self.accRawToMss

    def _needs_heartbeat(self, name: str) -> bool:
        return self._heartbeats.value.get(name, False) is True

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
                if self._board.send_RAW_msg(MSPy.MSPCodes['MSP_ACC_CALIBRATION'],data=[]):
                    dataHandler = self._board.receive_msg()
                    self._board.process_recv_data(dataHandler)
                # read IMU 10 times
                data = copy.deepcopy(self._default_accelerometer_calib)
                counter = 1
                num_points = 20
                
                for _ in range(num_points):
                    # noinspection PyBroadException
                    try:
                        self._board.fast_read_imu()
                        time.sleep(0.2)
                        
                        # Sometimes we receive a [0,0,0] SENSOR_DATA array, we need to skip this for
                        # averaging
                        if np.sum(self._board.SENSOR_DATA['accelerometer']) > 50:
                            data["x"] += self._board.SENSOR_DATA['accelerometer'][0]
                            data["y"] += self._board.SENSOR_DATA['accelerometer'][1]
                            data["z"] += self._board.SENSOR_DATA['accelerometer'][2]

                            counter += 1
                    except Exception as e:
                        self.logwarn(f"Could not calibrate IMU, received the following exception: {e}")
                        
                if counter < num_points * 0.5:
                    msg = f"After the calibration, we only received {counter - 1} datapoints " \
                          f"out of {num_points} expected. Aborting calibration..."
                    self.logwarn(msg)
                    return TriggerResponse(success=False, message=msg)
                # compute calibration
                calibration = {
                    "x": data["x"] / (counter),
                    "y": data["y"] / (counter),
                    "z": data["z"] / (counter)
                }
                with open(self._calib_file, "wt") as fout:
                    yaml.safe_dump(calibration, fout)
                # apply calibration
                self._accelerometer_calib = calibration
                self._apply_calibration()
                # ---
                print(f"IMU Calibration based on {counter - 1}/{num_points} datapoints:",
                      json.dumps(calibration, indent=4, sort_keys=True))

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

    def _read_battery_status(self):
        if self._board.send_RAW_msg(MSPy.MSPCodes['MSP_ANALOG'], data=[]):
            dataHandler = self._board.receive_msg()
            self._board.process_recv_data(dataHandler)

        if self._board.ANALOG is not None:
            if 'voltage' in self._board.ANALOG:
                voltage = self._board.ANALOG['voltage'] / 10.0  # reading scale
            else:
                self.logwarn("Unable to get Battery data: " + str(self._board.ANALOG))
                voltage = -1
        else:
            voltage = -1
            self.logwarn(f"Unable to get Battery data, analog is none")

        msg = BatteryState()
        msg.header = Header(stamp=rospy.Time.now())
        msg.voltage = voltage
        msg.present = True if voltage > 6.0 else False  # ~5V: power from Pi | 7V to 12.6V: power from battery
        return msg

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

                        # read battery status
                        if self._battery_reminder.is_time():
                            bat_msg = self._read_battery_status()
                            self._bat_pub.publish(bat_msg)

                        # update and send the flight commands to the board
                        self._compute_flight_commands()
                        self._send_flight_commands()

                        # publish the current mode
                        if self._last_published_mode != self._requested_mode:
                            self._mode_pub.publish(self._requested_mode)
                            self._last_published_mode = self._requested_mode
                    except FCError:
                        self.logwarn("Could not talk to the flight controller" + str(FCError))
                        continue

                # sleep for the remainder of the loop time
                self._clock.sleep()
        except Exception:
            traceback.print_exc()
        finally:
            self.loginfo('Shutdown received, disarming...')
            self._board.fast_msp_rc_cmd(self.rc_command(DroneMode.DISARMED))
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
                    # TODO: check if IDLE command is correct
                    self._command = self.rc_command(DroneMode.IDLE)

        if self._requested_mode is not DroneMode.ARMED:
            self._mode_change_counter = 0
    
    def _send_flight_commands(self):
        """ Send commands to the flight controller board """
        try:
            if self._board.send_RAW_RC(self._command):
                    dataHandler = self._board.receive_msg()
                    self._board.process_recv_data(dataHandler)
            # keep track of the last command sent
            if self._command != self._last_command:
                self._last_command = self._command
            # TODO: add aux1 and aux2 to DroneControl ros msg
            self._commands_pub.publish(DroneControl(
                self._command[0],
                self._command[1],
                self._command[2],
                self._command[3],
            ))
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
            if self._board.send_RAW_msg(MSPy.MSPCodes['MSP_MOTOR'], data=[]):
                        dataHandler = self._board.receive_msg()
                        self._board.process_recv_data(dataHandler)
            else:
                raise FCError(f"Unable to get MOTOR data, retry...")
        except Exception as e:
            self.logwarn(f"Unable to get MOTOR data {e}, retry...")
            raise FCError(f"Unable to get MOTOR data {e}, retry...")

        # self.logdebug(f"Retrieved MOTOR data: {self._board.MOTOR_DATA}")
        # create Motor message
        return DroneMotorCommand(
            header=Header(stamp=rospy.Time.now()),
            minimum=self._motor_command_range[0],
            maximum=self._motor_command_range[1],
            m1=int(self._board.MOTOR_DATA[0]),
            m2=int(self._board.MOTOR_DATA[1]),
            m3=int(self._board.MOTOR_DATA[2]),
            m4=int(self._board.MOTOR_DATA[3]),
        )
    
    def _get_pid_values_board(self):
        if self._board.send_RAW_msg(MSPy.MSPCodes['MSP_PIDNAMES'], data=[]):
            dataHandler = self._board.receive_msg()
            self._board.process_recv_data(dataHandler)
            self.loginfo(f'Received PID names from FC: {self._board.PIDNAMES}')

        if self._board.send_RAW_msg(MSPy.MSPCodes['MSP_PID'], data=[]):
            dataHandler = self._board.receive_msg()
            self._board.process_recv_data(dataHandler)
            self.loginfo(f'Received PID values from FC: {self._board.PIDs}')

        for name, pid_values in zip(self._board.PIDNAMES, self._board.PIDs):
            self.pids[name]['p'] = pid_values[0]
            self.pids[name]['i'] = pid_values[1]
            self.pids[name]['d'] = pid_values[2]

        ret = MultiWiiRpyPid(
            roll_p=self.pids['ROLL']['p'],
            roll_i=self.pids['ROLL']['i'],
            roll_d=self.pids['ROLL']['d'],
            pitch_p=self.pids['PITCH']['p'],
            pitch_i=self.pids['PITCH']['i'],
            pitch_d=self.pids['PITCH']['d'],
            yaw_p=self.pids['YAW']['p'],
            yaw_i=self.pids['YAW']['i'],
            yaw_d=self.pids['YAW']['d'],
        )  
        return ret
                
    def _read_imu_message(self):
        """
        Compute the ROS IMU message by reading data from the board.
        """
        try:
            # read roll, pitch, heading
            self._board.fast_read_attitude()
            # read lin_acc_x, lin_acc_y, lin_acc_z
            self._board.fast_read_imu()
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
        roll = np.deg2rad(self._board.SENSOR_DATA['kinematics'][0])
        pitch = -np.deg2rad(self._board.SENSOR_DATA['kinematics'][1])
        heading = np.deg2rad(self._board.SENSOR_DATA['kinematics'][2])
        # Note that at pitch angles near 90 degrees, the roll angle reading can fluctuate a lot
        # transform heading (similar to yaw) to standard math conventions, which
        # means angles are in radians and positive rotation is CCW
        heading = (-heading) % (2 * np.pi)

        # transform euler angles into quaternion
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, heading)
        # calculate the linear accelerations #TODO: check if the scaling is already performed in the MSPy library
        lin_acc_x = self._board.SENSOR_DATA['accelerometer'][0] * self.accRawToMss - self.accZeroX
        lin_acc_y = self._board.SENSOR_DATA['accelerometer'][1] * self.accRawToMss - self.accZeroY
        lin_acc_z = self._board.SENSOR_DATA['accelerometer'][2] * self.accRawToMss - self.accZeroZ

        # TODO: revisit this and use ROS' frames
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
        vid_pid_match = "VID:PID={}:{}".format(self._device["vendor_id"],
                                               self._device["product_id"])
        ports = serial_grep(vid_pid_match)
        devs = [p.device for p in ports]  # ['/dev/ttyUSB0', ...]
        self.loginfo(f"Devices matching VID:PID are: {devs}")
        # make sure we have at least one device
        if len(devs) <= 0:
            self.logfatal(f"Cannot find devices with properties: {self._device}.")
            sys.exit(1)
        # make sure we have no more than one device
        if len(devs) > 1:
            self.logfatal(f"Found {len(devs)} devices with properties: {self._device}.")
            sys.exit(2)
        # get device path
        dev = devs[0]

        # try connecting
        try:
            board = MSPy(dev).__enter__()
            if board == 1:
                raise SerialException
        except SerialException:
            self.logfatal(f"Cannot connect to the flight controller board at '{dev}'. "
                          f"The USB is unplugged. Please check connection.")
            sys.exit(3)
        # make sure we have a connection to the board
        if board is None:
            self.logfatal(f"The flight controller board could not be found at '{dev}'")
            sys.exit(4)
        # ---
        self._board = board

    def on_shutdown(self):
        """
        Disarm the drone and quits the flight controller node.
        """
        if self._board is None:
            return
        self.loginfo("Disarming!")
        self._command = self.rc_command(DroneMode.ARMED)
        self._switch_to_mode(DroneMode.ARMED, quiet=True)
        self._mode_pub.publish(DroneModeMsg(mode=DroneMode.DISARMED.value))
        rospy.sleep(0.5)
        sys.exit()


def main():
    # run flight controller driver communication loop
    fc = FlightController()
    fc.run()


if __name__ == '__main__':
    main()
