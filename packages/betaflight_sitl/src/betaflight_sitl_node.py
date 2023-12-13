#!/usr/bin/env python3

import threading
import time
import rospy
import subprocess

from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
# from duckietown_msgs.srv import StartBetaflightSITL, StartBetaflightSITLResponse


class BetaflightSITLNode(DTROS):


    def __init__(self):
        super(BetaflightSITLNode, self).__init__(node_name="betaflight_sitl_node", node_type=NodeType.DRIVER)
        # get parameters
        self._sitl_config = DTParam("~sitl", param_type=ParamType.DICT)

        # service
        # self._start_betaflight_sitl = rospy.Service("~start_betaflight_sitl", StartBetaflightSITL, self.srv_start_betaflight_sitl)
    
    def on_shutdown(self):
        pass

    def srv_start_betaflight_sitl(self, req):
        # start betaflight_sitl
        self._start_betaflight_sitl()
        # return StartBetaflightSITLResponse()

    def _start_betaflight_sitl(self, betaflight_ip, betaflight_path):
        self.loginfo("Waiting for betaflight SITL to start...")
        
        process = subprocess.Popen(['./betaflight_SITL.elf', betaflight_ip],
                                   cwd=betaflight_path,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE,
                                   universal_newlines=True)

        def read_output():
            target_string = "bind port 5761 for UART1"
            while True:
                output = process.stdout.readline().strip()
                print(output)  # Print the output if needed

                if target_string in output:
                    print("Found the target string:", target_string)
                    break  # Exit the loop if the target string is found

        output_thread = threading.Thread(target=read_output)
        output_thread.start()

        # Wait for the process to complete
        stdout, stderr = process.communicate()
        output_thread.join()

        # Retrieve the final output after the process completes
        print(stdout)
        print(stderr)

if __name__ == "__main__":
    node = BetaflightSITLNode()
    node._start_betaflight_sitl('127.0.0.1', '/usr/bin/betaflight')
    rospy.spin()
