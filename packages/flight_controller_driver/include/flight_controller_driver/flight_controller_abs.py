from abc import ABC, abstractmethod
from collections import defaultdict
import json
import traceback
from enum import IntEnum
from typing import List, Literal, Optional, Tuple

from dataclasses import dataclass

from duckietown_messages.actuators.drone_parameters import FlightControllerParameters
import numpy as np
import sys
import logging

from serial import SerialException

from yamspy import MSPy

RAW_GYRO_TO_DEG_S = 1 / 16.4

class DroneMode(IntEnum):
    DISARMED = 0
    ARMED = 1
    FLYING = 2
    IDLE = 3

# read m1, m2, m3, m4
@dataclass
class MotorState:
    m1: int
    m2: int
    m3: int
    m4: int

@dataclass
class AttitudePidGains:
    roll_p: int
    roll_i: int
    roll_d: int

    pitch_p: int
    pitch_i: int
    pitch_d: int

    yaw_p: int
    yaw_i: int
    yaw_d: int
    
    @staticmethod
    def from_parameters_message(msg : FlightControllerParameters):
        """
        Convert a DroneMotorCommand message to an AttitudePidGains object.
        """
        return AttitudePidGains(
            roll_p=msg.roll_pid_kp,
            roll_i=msg.roll_pid_ki,
            roll_d=msg.roll_pid_kd,
            pitch_p=msg.pitch_pid_kp,
            pitch_i=msg.pitch_pid_ki,
            pitch_d=msg.pitch_pid_kd,
            yaw_p=msg.yaw_pid_kp,
            yaw_i=msg.yaw_pid_ki,
            yaw_d=msg.yaw_pid_kd
        )
        
    def to_parameters_message(self):
        """
        Convert the AttitudePidGains object to a DroneMotorCommand message.
        """
        return FlightControllerParameters(
            roll_pid_kp=self.roll_p,
            roll_pid_ki=self.roll_i,
            roll_pid_kd=self.roll_d,
            pitch_pid_kp=self.pitch_p,
            pitch_pid_ki=self.pitch_i,
            pitch_pid_kd=self.pitch_d,
            yaw_pid_kp=self.yaw_p,
            yaw_pid_ki=self.yaw_i,
            yaw_pid_kd=self.yaw_d
        )

@dataclass
class SerialConfig:
    port: str
    baudrate: Optional[int] = 115200

@dataclass
class Mode2RC:
    """Map of modes to RC commands values
    """
    disarm: List[int]
    arm: List[int]
    idle: List[int]
    flying: Optional[List[int]] = None

    def from_mode(self, mode : DroneMode):
        """
        Get the RC command for a given mode.
        """
        mode_to_rc = {
            DroneMode.DISARMED: self.disarm,
            DroneMode.ARMED: self.arm,
            DroneMode.FLYING: self.flying,
            DroneMode.IDLE: self.idle
        }
        return mode_to_rc[mode]



class FCError(SerialException):
    pass


class FlightControllerAbs(ABC):
    """
    A class that implements the communication channels with the flight controller board via USB.
    It reads data from the IMU and the PWM signals going to the motors while relaying the command
    [R, P, Y, T] provided via ROS.

    """

    def __init__(self, mode_to_rc_commands : Mode2RC):
        super(FlightControllerAbs, self).__init__()

        self._mode_to_rc_commands = mode_to_rc_commands

        self.pids = defaultdict(dict)
        self._desired_pids = {}

        # (try to) connect to the flight controller board
        self._board: Optional[MSPy] = None
        self.setup()
        self.connect()

        if self._board is None:
            return
        
        # Accelerometer scaling (from IMU values to m/s, IMU are in [g/512], 1g = 9.81 m/s^2)
        self.ACCELEROMETER_SCALING_FACTOR = 9.81/512
        
        # yaw offset
        self.yaw_offset = 0.0

        # store the command to send to the flight controller, initialize as disarmed
        self._command = self.mode_to_rc_command(DroneMode.DISARMED)
        self._last_command = self.mode_to_rc_command(DroneMode.DISARMED)


    def mode_to_rc_command(self, mode: DroneMode) -> Optional[List[int]]:
        """
        Provides the raw RC commands for a given mode.

        Args:
            mode: Mode to retrieve the raw RC command for.

        Returns:
            Raw RC command to send to the flight controller to trigger the given mode.

        """
        return self._mode_to_rc_commands.from_mode(mode)


    def calibrate_imu(self, _):
        """ Calibrate IMU """
        logging.info("Calibrating IMU...")
        if self._board is not None:
            if self._board.send_RAW_msg(MSPy.MSPCodes['MSP_ACC_CALIBRATION'],data=[]):
                logging.debug("Sent accelerometer calibration message")
                dataHandler = self._board.receive_msg()
                logging.debug("Received accelerometer calibration response message")
                self._board.process_recv_data(dataHandler)
                logging.info("IMU Calibration performed")

            else:
                raise FCError("Unable to calibrate IMU, retry...")
        else:
            traceback.print_exc()
            raise FCError("Unable to connect to the flight controller board, retry...")

    def send_command(self, command):
        """ Send commands to the flight controller board """
        try:
            self._send_rc_to_board(command)
        except Exception as e:
            logging.error(f"Error communicating with board {e}")

    def disarm(self):
        """ Disarm the drone """
        self.send_command(self.mode_to_rc_command(DroneMode.DISARMED))

    def arm(self):
        """ Arm the drone """
        self.send_command(self.mode_to_rc_command(DroneMode.ARMED))

    def idle(self):
        """ Put the drone in idle mode """
        self.send_command(self.mode_to_rc_command(DroneMode.IDLE))
    
    def fly(self):
        """ Put the drone in flying mode """
        self.send_command(self.mode_to_rc_command(DroneMode.FLYING))

    @abstractmethod
    def _send_rc_to_board(self, rc_command):
        pass

    @abstractmethod
    def setup(self):
        """
        Implement this method to setup the flight controller board, run before connecting.
        """

    @property
    def voltage(self):
        if self._board.send_RAW_msg(MSPy.MSPCodes['MSP_ANALOG'], data=[]):
            dataHandler = self._board.receive_msg()
            self._board.process_recv_data(dataHandler)

        if self._board.ANALOG is not None:
            if 'voltage' in self._board.ANALOG:
                voltage = self._board.ANALOG['voltage']
            else:
                logging.warning("Unable to get Battery data: " + str(self._board.ANALOG))
                voltage = -1
        else:
            voltage = -1
            logging.warning("Unable to get Battery data, ANALOG is none")

        return voltage
   
    @property
    def motors_pwm(self) -> Tuple[int, int, int, int]:
        """
        Reads the motors signals sent by the flight controller to the ESCs.
        
        Returns the pwm duty cycle of each motor.

        Returns:
            m1, m2, m3, m4
        """
        if self._board.send_RAW_msg(MSPy.MSPCodes['MSP_MOTOR'], data=[]):
            dataHandler = self._board.receive_msg()
            self._board.process_recv_data(dataHandler)
        else:
            raise FCError("Unable to get MOTOR data, retry...")

        m1=int(self._board.MOTOR_DATA[0])
        m2=int(self._board.MOTOR_DATA[1])
        m3=int(self._board.MOTOR_DATA[2])
        m4=int(self._board.MOTOR_DATA[3])

        return m1, m2, m3, m4

    @property
    def attitude_pid_gains(self):
        if self._board.send_RAW_msg(MSPy.MSPCodes['MSP_PIDNAMES'], data=[]):
            dataHandler = self._board.receive_msg()
            self._board.process_recv_data(dataHandler)
            logging.info(f'Received PID names from FC: {self._board.PIDNAMES}')

        if self._board.send_RAW_msg(MSPy.MSPCodes['MSP_PID'], data=[]):
            dataHandler = self._board.receive_msg()
            self._board.process_recv_data(dataHandler)
            logging.info(f'Received PID values from FC: {self._board.PIDs}')

        for name, pid_values in zip(self._board.PIDNAMES, self._board.PIDs):
            self.pids[name]['p'] = pid_values[0]
            self.pids[name]['i'] = pid_values[1]
            self.pids[name]['d'] = pid_values[2]

        ret = AttitudePidGains(
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
    
    @property
    def attitude(self) -> Tuple[float, float, float]:
        """
        Attitude values of the drone. In degrees.
        
        Returns:
            roll, pitch, yaw
        """
        try:
            # read roll, pitch, heading
            self._board.fast_read_attitude()
        except Exception as e:
            logging.warning(f"Unable to get attitude data {e}, retry...")
            raise FCError(f"Unable to get attitude data {e}, retry...")
        
        # calculate values to update imu_message:
        roll = np.deg2rad(self._board.SENSOR_DATA['kinematics'][0])
        pitch = np.deg2rad(self._board.SENSOR_DATA['kinematics'][1])
        yaw = np.deg2rad(self._board.SENSOR_DATA['kinematics'][2])
        
        # TODO: check if the negative sign is correct
        yaw = (-yaw) % (2 * np.pi) - self.yaw_offset

        return roll, pitch, yaw
    
    @property
    def acceleration(self) -> Tuple[float, float, float]:
        """
        Acceleration values of the drone. In [m/s].
        
        Returns:
            a_x, a_y, a_z
        """
        if self._board is not None:
            try:
                # read lin_acc_x, lin_acc_y, lin_acc_z
                self._board.fast_read_imu()
            except Exception as e:
                logging.warning(f"Unable to get IMU data {e}, retry...")
                raise FCError(f"Unable to get IMU data {e}, retry...")

            # calculate the linear accelerations
            a_x = self._board.SENSOR_DATA['accelerometer'][0] * self.ACCELEROMETER_SCALING_FACTOR
            a_y = self._board.SENSOR_DATA['accelerometer'][1] * self.ACCELEROMETER_SCALING_FACTOR
            a_z = self._board.SENSOR_DATA['accelerometer'][2] * self.ACCELEROMETER_SCALING_FACTOR
        
        else:
            raise FCError("Unable to connect to the flight controller board, retry...")

        # TODO: we should revisit this and use ROS' frames
        # Rotate the IMU frame to align with our convention for the drone's body
        # frame. IMU: x is forward, y is left, z is up. We want: x is right,
        # y is forward, z is up.
        lin_acc_x_drone_body = -a_y
        lin_acc_y_drone_body = a_x
        lin_acc_z_drone_body = a_z

        roll, pitch, _ = self.attitude

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

        return a_x, a_y, a_z

    @property
    def gyro(self) -> Tuple[float, float, float]:
        """
        Angular velocities measured by the gyroscope of the drone. In [deg/s].
        
        Note: this function converts from raw values to degrees per second,
        the scaling factor is `RAW_GYRO_TO_DEG_S`, which is 1/16.4 for the MPU6050 and BMI270.
        If the flight controller uses a different IMU, this value should be updated.

        Returns:
            omega_x, omega_y, omega_z
        """
        if self._board is not None:
            try:
                # read IMU RAW data
                self._board.fast_read_imu()
            except Exception as e:
                logging.warning(f"Unable to get IMU data {e}, retry...")
                raise FCError(f"Unable to get IMU data {e}, retry...")

            # calculate the linear accelerations
            omega_x = self._board.SENSOR_DATA['gyroscope'][0] * RAW_GYRO_TO_DEG_S
            omega_y = self._board.SENSOR_DATA['gyroscope'][1] * RAW_GYRO_TO_DEG_S
            omega_z = self._board.SENSOR_DATA['gyroscope'][2] * RAW_GYRO_TO_DEG_S
            
            return omega_x, omega_y, omega_z
        
        else:
            raise FCError("Unable to connect to the flight controller board, retry...")

    def connect(self):
        """ Connect to the flight controller board """
        dev = self._get_board_device()

        # try connecting
        try:
            board = MSPy(dev, logfilename='/tmp/MSPy.log').__enter__()
            if board == 1:
                raise SerialException
        except SerialException:
            logging.critical(f"Cannot connect to the flight controller board at '{dev}'. "
                          f"The USB is unplugged. Please check connection.")
            sys.exit(3)
        # make sure we have a connection to the board
        if board is None:
            logging.critical(f"The flight controller board could not be found at '{dev}'")
            sys.exit(4)
        # ---
        self._board = board

    @abstractmethod
    def _get_board_device(self) -> str:
        pass

    def _update_buffer_desired_pids(
        self,
        axis_name: Literal['ROLL', 'PITCH', 'YAW', 'ALT', 'Pos', 'PosR', 'NavR', 'LEVEL', 'MAG', 'VEL'],
        component_name: Literal['p', 'i', 'd'],
        coefficient_value: int,
    ):
        """ Before sending all desired PIDs to the FC, buffer locally in this python obj """
        if len(self.pids) == 0:
            _ = self.attitude_pid_gains

        if len(self._desired_pids) == 0 and len(self.pids) != 0:
            self._desired_pids = self.pids.copy()

        if len(self._desired_pids) == 0:
            print("Not updating desired PIDs because the original values have not been obtained")
            return

        self._desired_pids[axis_name][component_name] = coefficient_value

    def _update_pids(self):
        if len(self._desired_pids) == 0:
            print("Not sending desired PIDs to the FC. The desired PIDs are empty")
            return False

        try:
            desired = []
            for per_item_pid in self._desired_pids.values():
                for k in ['p', 'i', 'd']:
                    desired.append(per_item_pid[k])

            if self._board.send_RAW_msg(MSPy.MSPCodes['MSP_SET_PID'], data=desired):
                dataHandler = self._board.receive_msg()
                self._board.process_recv_data(dataHandler)
                logging.info(f'Received PID values from FC: {self._board.PIDs}')

            # update self.pids
            _ = self.attitude_pid_gains

            for key_name, per_item_pid in self.pids.items():
                assert self._desired_pids[key_name]['p'] == per_item_pid['p']
                assert self._desired_pids[key_name]['i'] == per_item_pid['i']
                assert self._desired_pids[key_name]['d'] == per_item_pid['d']

            print("================================================================")
            print("PID update successful. After setting desired PIDs, the new PIDs are:")
            print(json.dumps(self.pids, indent=2))
            print("================================================================")

            return True
        
        except Exception as e:
            print(f"Failed to update PIDs to desired. Error: {e}")
            return False

    def set_pids_rpy(
        self,
        roll_p: Optional[int] = None, roll_i: Optional[int] = None, roll_d: Optional[int] = None,
        pitch_p: Optional[int] = None, pitch_i: Optional[int] = None, pitch_d: Optional[int] = None,
        yaw_p: Optional[int] = None, yaw_i: Optional[int] = None, yaw_d: Optional[int] = None,
    ) -> bool:
        """
        Set the PID values for the roll, pitch, and yaw axes.
        
        Returns:
            True if the PID values were successfully updated, False otherwise.    
        """

        if roll_p is not None:
            self._update_buffer_desired_pids('ROLL', 'p', roll_p)
        if roll_i is not None:
            self._update_buffer_desired_pids('ROLL', 'i', roll_i)
        if roll_d is not None:
            self._update_buffer_desired_pids('ROLL', 'd', roll_d)

        if pitch_p is not None:
            self._update_buffer_desired_pids('PITCH', 'p', pitch_p)
        if pitch_i is not None:
            self._update_buffer_desired_pids('PITCH', 'i', pitch_i)
        if pitch_d is not None:
            self._update_buffer_desired_pids('PITCH', 'd', pitch_d)

        if yaw_p is not None:
            self._update_buffer_desired_pids('YAW', 'p', yaw_p)
        if yaw_i is not None:
            self._update_buffer_desired_pids('YAW', 'i', yaw_i)
        if yaw_d is not None:
            self._update_buffer_desired_pids('YAW', 'd', yaw_d)

        return self._update_pids()

