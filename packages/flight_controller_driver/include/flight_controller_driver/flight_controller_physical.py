from dataclasses import dataclass
import logging
import sys
from typing import List
from .flight_controller_abs import FlightControllerAbs, Mode2RC

from serial.tools.list_ports import grep as serial_grep

@dataclass
class Device:
    vendor_id: str
    product_id: str

class FlightControllerPhysical(FlightControllerAbs):
    def __init__(self, mode_to_rc_commands: Mode2RC, device_ids : Device, ):
        self._device : Device = device_ids
        super().__init__(mode_to_rc_commands)

        
    def _get_board_device(self) -> str:
        vid_pid_match = "VID:PID={}:{}".format(self._device.vendor_id,
                                            self._device.product_id)
        ports = serial_grep(vid_pid_match)
        devs = [p.device for p in ports]  # ['/dev/ttyUSB0', ...]
        logging.info(f"Devices matching VID:PID are: {devs}")
        # make sure we have at least one device
        if len(devs) <= 0:
            logging.critical(f"Cannot find devices with properties: {self._device}.")
            sys.exit(1)
        # make sure we have no more than one device
        if len(devs) > 1:
            logging.critical(f"Found {len(devs)} devices with properties: {self._device}.")
            sys.exit(2)
        # get device path
        dev = devs[0]
        return dev
    
    def _send_rc_to_board(self, rc_command: List[int]):
        """ Send RC command to the flight controller board """
        assert len(rc_command) == 6, f"RC command must have 6 elements, has {len(rc_command)}: {rc_command}."
        rc_command.extend([1000, 1000])
        self._board.send_raw_command(len(rc_command), self._board.SET_RAW_RC, rc_command)
        self._board.receiveDataPacket()
        
        
    def setup(self):
        return super().setup()
