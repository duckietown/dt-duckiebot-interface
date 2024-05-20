from dataclasses import dataclass
import logging
import sys
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
    
    def _send_rc_to_board(self, rc_command):
        """ Send RC command to the flight controller board """
        self._board.fast_msp_rc_cmd(rc_command)
        
    def setup(self):
        return super().setup()
