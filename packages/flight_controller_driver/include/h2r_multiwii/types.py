from dataclasses import dataclass


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
