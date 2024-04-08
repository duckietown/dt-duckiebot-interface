from dataclasses import dataclass
import logging
import time
import subprocess
from threading import Thread

from .sitl.utils import RCPacket, SocketBetaflight
from .flight_controller_abs import FCError, FlightControllerAbs, Mode2RC
from flight_controller_driver.flight_controller_abs import SerialConfig

@dataclass
class SITLConfig:
    ip: str
    msp_port: int
    rc_port: int

class FlightControllerVirtual(FlightControllerAbs):
    def __init__(self, mode_to_rc_commands : Mode2RC, serial_config : SerialConfig, sitl_config : SITLConfig):
        self._virtual_serial : SerialConfig = serial_config
        self._sitl_config : SITLConfig = sitl_config

        super().__init__(mode_to_rc_commands)
    
    def _pipe_tcp_into_serial(self,serial_port : str, sitl_ip : str, sitl_port : int):
        command = ["socat", "-dd", f"pty,link={serial_port},raw,echo=0",
                                      f"tcp:{sitl_ip}:{sitl_port}"]
        try:
            run_command_in_thread(' '.join(command))
            time.sleep(1)
        except:
            logging.error(f"Failed to run command: {' '.join(command)}")
            raise

    def _get_board_device(self) -> str:
        return self._virtual_serial.port

    def setup(self):
        super().setup()
        self._dev = self._start_betaflight_sitl()

    def _start_betaflight_sitl(self,) -> str:
        serial_port_path = self._virtual_serial.port

        sitl_ip = self._sitl_config.ip
        sitl_msp_port = self._sitl_config.msp_port
        sitl_rc_port = self._sitl_config.rc_port

        # Pipe the SITL tcp port to a virtual serial port.
        self._pipe_tcp_into_serial(
                serial_port_path,
                sitl_ip,
                sitl_msp_port
                )
        
        # Connect to the virtual RC port.
        self._connect_to_virtual_rc(
            sitl_ip,
            sitl_rc_port
            )
        
        return serial_port_path

    def _send_rc_to_board(self, rc_command):
        """The rc command is processed and sent to the board via the virtual RC port.

        Args:
            rc_command (list): 6 element list of integers representing the RC command [0-2000].
        """
        pkt = RCPacket()
        pkt.channels[0:6] = rc_command
        # Swap yaw and throttle AERT1234->AETR1234 as expected by the RC input of the SITL.
        pkt.channels[2], pkt.channels[3] = pkt.channels[3], pkt.channels[2]

        self._virtual_rc.udp_send(pkt.to_bytes())
    
    def _connect_to_virtual_rc(self, ip, port):
        self._virtual_rc = SocketBetaflight(ip, port)
        if self._virtual_rc.init():
            logging.error("Not able to open socket")
            raise FCError("Not able to open socket")



def execute_command(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    while True:
        output = process.stdout.readline()
        if output == b'' and process.poll() is not None:
            break
        if "starting data transfer loop" in output:
            print("SITL connection established")
        if output:
            print(output.decode().strip())  # Print the output of the command

def run_command_in_thread(command):
    thread = Thread(target=execute_command, args=(command,))
    thread.start()
