from abc import ABC, abstractmethod
from collections import defaultdict
import json
import traceback
from enum import IntEnum
from typing import List, Literal, Optional, Tuple

from dataclasses import dataclass

# from duckietown_messages.actuators.attitude_pids_parameters import AttitudePIDParameters
import numpy as np
import sys
import logging

from serial import SerialException

from .h2rMultiWii import MultiWii

RAW_GYRO_TO_DEG_S = 1 / 16.4

class DroneMode(IntEnum):
    DISARMED = 0
    ARMED = 1
    FLYING = 2

# read m1, m2, m3, m4
@dataclass
class MotorState:
    m1: int
    m2: int
    m3: int
    m4: int

# @dataclass
# class AttitudePidGains:
#     roll_p: float
#     roll_i: float
#     roll_d: float

#     pitch_p: float
#     pitch_i: float
#     pitch_d: float

#     yaw_p: float
#     yaw_i: float
#     yaw_d: float
    
#     @staticmethod
#     def from_parameters_message(msg : AttitudePIDParameters):
#         """
#         Convert a DroneMotorCommand message to an AttitudePidGains object.
#         """
#         return AttitudePidGains(
#             roll_p=msg.roll_pid_kp,
#             roll_i=msg.roll_pid_ki,
#             roll_d=msg.roll_pid_kd,
#             pitch_p=msg.pitch_pid_kp,
#             pitch_i=msg.pitch_pid_ki,
#             pitch_d=msg.pitch_pid_kd,
#             yaw_p=msg.yaw_pid_kp,
#             yaw_i=msg.yaw_pid_ki,
#             yaw_d=msg.yaw_pid_kd
#         )

#     def to_parameters_message(self):
#         """
#         Convert the AttitudePidGains object to a DroneMotorCommand message.
#         """
#         return AttitudePIDParameters(
#             roll_pid_kp=self.roll_p,
#             roll_pid_ki=self.roll_i,
#             roll_pid_kd=self.roll_d,
#             pitch_pid_kp=self.pitch_p,
#             pitch_pid_ki=self.pitch_i,
#             pitch_pid_kd=self.pitch_d,
#             yaw_pid_kp=self.yaw_p,
#             yaw_pid_ki=self.yaw_i,
#             yaw_pid_kd=self.yaw_d
#         )

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
    flying: Optional[List[int]] = None

    def from_mode(self, mode : DroneMode):
        """
        Get the RC command for a given mode.
        """
        mode_to_rc = {
            DroneMode.DISARMED: self.disarm,
            DroneMode.ARMED: self.arm,
            DroneMode.FLYING: self.flying,
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
        self.setup()
        self._board: MultiWii = self.connect()

        if self._board is None:
            return
        
        # Accelerometer scaling (from IMU values to m/s, IMU are in [g/512], 1g = 9.81 m/s^2)
        self.ACCELEROMETER_SCALING_FACTOR = 9.81/512
        
        # yaw offset
        self.yaw_offset_degrees = 0.0

    def mode_to_rc_command(self, mode: DroneMode) -> List[int]:
        """
        Provides the raw RC commands for a given mode.

        Args:
            mode: Mode to retrieve the raw RC command for.

        Returns:
            Raw RC command to send to the flight controller to trigger the given mode.

        """
        return self._mode_to_rc_commands.from_mode(mode)

    def calibrate_imu(self,):
        """ Calibrate IMU """
        logging.info("Calibrating IMU...")
        if self._board is not None:
            self._board.calibrate()
        else:
            traceback.print_exc()
            raise FCError("Unable to connect to the flight controller board, retry...")

    async def send_command(self, command):
        """ Send commands to the flight controller board """
        print(f"Sending command: {command}")
        try:
            await self._send_rc_to_board(command)
        except Exception as e:
            logging.error(f"Error communicating with board {e}")

    def disarm(self):
        """ Disarm the drone """
        self.send_command(self.mode_to_rc_command(DroneMode.DISARMED))

    def arm(self):
        """ Arm the drone """
        self.send_command(self.mode_to_rc_command(DroneMode.ARMED))
    
    def fly(self):
        """ Put the drone in flying mode """
        self.send_command(self.mode_to_rc_command(DroneMode.FLYING))

    @abstractmethod
    async def _send_rc_to_board(self, rc_command):
        pass

    @abstractmethod
    def setup(self):
        """
        Implement this method to setup the flight controller board, executed prior connecting to the serial port.
        """

    @property
    async def voltage(self):
        """
        Get the voltage of the flight controller board.

        Returns:
            The voltage of the flight controller board.
        """
        await self._board.update_battery()

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
        await self._board.update_battery()

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
    async def motors_pwm(self) -> Tuple[int, int, int, int]:
        """
        Reads the motors signals sent by the flight controller to the ESCs.
        
        Returns the pwm duty cycle of each motor.

        Returns:
            m1, m2, m3, m4
        """
        await self._board.getData(MultiWii.MOTOR)

        m1=int(self._board.motor_data()[0])
        m2=int(self._board.motor_data()[1])
        m3=int(self._board.motor_data()[2])
        m4=int(self._board.motor_data()[3])

        return m1, m2, m3, m4
    
    @property
    async def attitude(self) -> Tuple[float, float, float]:
        """
        Attitude values of the drone. In degrees.
        
        Returns:
            roll, pitch, yaw
        """
        try:
            # read roll, pitch, heading
            await self._board.getData(MultiWii.ATTITUDE)
        except Exception as e:
            logging.warning(f"Unable to get attitude data {e}, retry...")
            raise FCError(f"Unable to get attitude data {e}, retry...")
        
        # calculate values to update imu_message:
        roll = self._board.attitude['angx']
        pitch = self._board.attitude['angy']
        yaw = self._board.attitude['heading']
        
        # TODO: check if the negative sign is correct
        yaw -= self.yaw_offset_degrees

        return roll, pitch, yaw

    async def read_imu_values(self):
        """
        Read IMU values from the board and store them in the object.

        They can be accessed by the methods:
        - acceleration
        - gyro
        """
        if self._board is not None:
            try:
                # read lin_acc_x, lin_acc_y, lin_acc_z
                await self._board.getData(MultiWii.RAW_IMU)
            except Exception as e:
                logging.warning(f"Unable to get IMU data {e}, retry...")
                raise FCError(f"Unable to get IMU data {e}, retry...")
        else:
            raise FCError("Unable to connect to the flight controller board, retry...")
       
    @property
    def acceleration(self) -> Tuple[float, float, float]:
        """
        Acceleration values of the drone. In [m/s^2].
        
        Returns:
            a_x, a_y, a_z
        """
        # calculate the linear accelerations
        a_x = self._board.rawIMU['ax'] * self.ACCELEROMETER_SCALING_FACTOR
        a_y = self._board.rawIMU['ay'] * self.ACCELEROMETER_SCALING_FACTOR
        a_z = self._board.rawIMU['az'] * self.ACCELEROMETER_SCALING_FACTOR   

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
        # calculate the linear accelerations
        omega_x = self._board.rawIMU['gx'] * RAW_GYRO_TO_DEG_S
        omega_y = self._board.rawIMU['gy'] * RAW_GYRO_TO_DEG_S
        omega_z = self._board.rawIMU['gz'] * RAW_GYRO_TO_DEG_S
        
        return omega_x, omega_y, omega_z

    async def zero_yaw(self):
        """
        Zero the yaw angle of the drone.
        """
        self.yaw_offset_degrees = await self.attitude[2]

    def connect(self) -> MultiWii:
        """ Connect to the flight controller board """
        dev = self._get_board_device()

        # try connecting
        try:
            board = MultiWii()
        except SerialException:
            logging.critical(f"Cannot connect to the flight controller board at '{dev}'. "
                          f"The USB is unplugged. Please check connection.")
            sys.exit(3)
        # make sure we have a connection to the board
        if board is None:
            logging.critical(f"The flight controller board could not be found at '{dev}'")
            sys.exit(4)
        # ---
        return board

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
        return True

    def set_pids_rpy(
        self,
        roll_p: Optional[int] = None, roll_i: Optional[int] = None, roll_d: Optional[int] = None,
        pitch_p: Optional[int] = None, pitch_i: Optional[int] = None, pitch_d: Optional[int] = None,
        yaw_p: Optional[int] = None, yaw_i: Optional[int] = None, yaw_d: Optional[int] = None,
    ) -> bool:
        return True
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

    @property
    def attitude_pid_gains(self):
        return False
