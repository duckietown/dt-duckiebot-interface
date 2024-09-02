from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Tuple

from serial import SerialException


from dt_node_utils.config import NodeConfiguration

class FCError(SerialException):
    pass

class DroneMode(IntEnum):
    DISARMED = 0
    ARMED = 1
    FLYING = 2

@dataclass
class SerialConfig:
    port: str
    baudrate: Optional[int] = 115200

@dataclass
class Device:
    vendor_id: str
    product_id: str

@dataclass
class SITLConfig:
    ip: str
    msp_port: int
    rc_port: int

@dataclass
class FlightControllerConfiguration(NodeConfiguration):
    @dataclass
    class Frequency:
        commands: int
        imu: int
        motors: int
        battery: int

    @dataclass
    class Heartbeats:
        altitude: bool
        joystick: bool
        pid: bool
        state_estimator: bool

    @dataclass
    class RCCommands:
        disarm: List[int]
        arm: List[int]
        flying: List[int]

    frequency: Frequency
    heartbeats: Heartbeats
    motor_command_range: Tuple[int, int]
    rc_commands: RCCommands
    serial: SerialConfig
    device: Optional[Device] = None
    sitl: Optional[SITLConfig] = None