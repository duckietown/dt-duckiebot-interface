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
from flight_controller_driver import FlightController

def main():
    # run flight controller driver communication loop
    fc = FlightController()
    fc.run()


if __name__ == '__main__':
    main()
