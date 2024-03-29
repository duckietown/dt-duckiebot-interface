#!/usr/bin/env python3

import sys
import signal

from duckietown.dtros import DTROS, NodeType
from dt_robot_rest_api import RobotRestAPI

# battery test
import requests
# wifi test
import subprocess
import re

from hardware_test_robot_host import HardwareTestWifi, HardwareTestBattery


class RobotRestAPInode(DTROS):
    def __init__(self):
        super(RobotRestAPInode, self).__init__(
            node_name="robot_http_api_node", node_type=NodeType.INFRASTRUCTURE, dt_ghost=True
        )

        # user hardware tests
        self._hardware_test_battery = HardwareTestBattery()
        self._hardware_test_wifi = HardwareTestWifi()


def signal_handler(sig, frame):
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    ros_node = RobotRestAPInode()
    api = RobotRestAPI(debug=False)
    api.run(host="0.0.0.0")
