from dataclasses import dataclass
from typing import List, Optional, Tuple

from flight_controller_driver.flight_controller_abs import SerialConfig
from flight_controller_driver.flight_controller_virtual import SITLConfig
from flight_controller_driver.flight_controller_physical import Device

from dt_node_utils.config import NodeConfiguration

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
        idle: List[int]
        flying: List[int]

    frequency: Frequency
    heartbeats: Heartbeats
    motor_command_range: Tuple[int, int]
    rc_commands: RCCommands
    serial: SerialConfig
    device: Optional[Device] = None
    sitl: Optional[SITLConfig] = None