import rospy
import subprocess
from threading import Thread

from .flight_controller_physical import FlightController

class FlightControllerSITL(FlightController):
    def __init__(self):
        virtual_serial_config = rospy.get_param("~virtual_serial")
        self._virtual_serial_port = virtual_serial_config['port']
        self._sitl_ip = virtual_serial_config['sitl_ip']
        self._sitl_port = virtual_serial_config['sitl_port']
        super().__init__()
        
    def _pipe_tcp_into_serial(serial_port : str, sitl_ip : str, sitl_port : int):
        command = ["socat", "-dd", f"pty,link={serial_port},raw,echo=0",
                                      f"tcp:{sitl_ip}:{sitl_port}"]
        run_command_in_thread(' '.join(command))

    def _get_board_device(self) -> str:
        self._pipe_tcp_into_serial(
                self._virtual_serial_port,
                self._sitl_ip,
                self._sitl_port
                )

        return self._virtual_serial_port

    def _start_betaflight_sitl(self):
        """Start the betaflight SITL process and pipe the tcp port to a virtual serial port.
        """
        pass


def execute_command(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    while True:
        output = process.stdout.readline()
        if output == b'' and process.poll() is not None:
            break
        if output:
            print(output.decode().strip())  # Print the output of the command

def run_command_in_thread(command):
    thread = Thread(target=execute_command, args=(command,))
    thread.start()
