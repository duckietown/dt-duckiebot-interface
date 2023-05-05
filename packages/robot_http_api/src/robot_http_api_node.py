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

from abc import ABC, abstractmethod
from typing import Optional, Dict
from std_srvs.srv import Trigger, TriggerResponse
import rospy


class HWTest(ABC):
    @abstractmethod
    def test_id(self) -> str:
        """Short name, used to report start and end of test"""
        pass

    @abstractmethod
    def test_desc_preparation(self) -> str:
        """Preparation before running. E.g. put the DB upside down"""
        pass

    def test_desc_running(self) -> str:
        """Actual steps to run the test"""
        # default: just click the "Run test" button
        return "Run the test"

    @abstractmethod
    def test_desc_expectation(self) -> str:
        """Expected outcome"""
        pass

    @abstractmethod
    def test_desc_log_gather(self) -> str:
        """How to gather logs before reporting"""
        pass

    def test_desc(self) -> str:
        """Test description and key params"""
        # TODO: use JSON and keys to separate sections
        return "\n\n".join([
            self.test_desc_preparation(),
            self.test_desc_expectation(),
            self.test_desc_running(),
            self.test_desc_log_gather(),
        ])

    def srv_cb_tst_desc(self, _):
        return TriggerResponse(
            success=True,
            message=self.test_desc(),
        )

    @abstractmethod
    def run_test(self) -> Optional[bool]:  # TODO: decide whether auto grade or not
        """return True or False if the result could be determined within the test"""
        pass


class HWTestWifi(HWTest):
    def __init__(self) -> None:
        super().__init__()
        self._desc_tst_srv = rospy.Service('~tests/wifi/desc', Trigger, self.srv_cb_tst_desc)
        self._tst_srv = rospy.Service('~tests/wifi/run', Trigger, self._tst)
        # test settings

    def _tst(self, _):
        success = True

        try:
            result = subprocess.run(["ifconfig", "wlan0"], capture_output=True, text=True)

            if result.stderr == "":
                # Define a regular expression pattern to match the IP address
                ip_pattern = r'inet (\d+\.\d+\.\d+\.\d+)'
                # Search the output using the regex pattern
                match = re.search(ip_pattern, result.stdout)
                # Extract the IP address from the match object, or set it to None if the interface is down
                ip_address = match.group(1) if match and 'UP' in result.stdout else None

                response = f"IP address of wlan0: {ip_address}"
            else:
                response = result.stderr
        except Exception as e:
            response = f"Error running test. {e}"
            success = False

        # Return the service response
        return TriggerResponse(success=success, message=response)

    def test_id(self) -> str:
        return f"Wifi"

    def test_desc_preparation(self) -> str:
        return "Make sure the USB Wifi dongle is plugged in to your Duckiebot, and it is blinking."

    def test_desc_expectation(self) -> str:
        return "The IP address of the `wlan0' network interface should be displayed below."

    def test_desc_log_gather(self) -> str:
        return (
            "On your laptop, run the following command to save the logs.\n"
            "Replace the `[path/to/save]' to the directory path where you would like to save the logs.\n"
            "`ssh duckie@[your_Duckiebot_hostname].local ifconfig > [path/to/save/]logs-network-ifconfig.txt'\n"
            "(You might need to provide the password to your Duckiebot when prompted.)"
        )

    def test_params(self) -> str:
        return f"[{self.test_id()}] N/A"

    def run_test(self) -> Optional[bool]:
        pass


class HWTestBattery(HWTest):
    def __init__(self) -> None:
        super().__init__()
        self._desc_tst_srv = rospy.Service('~tests/battery/desc', Trigger, self.srv_cb_tst_desc)
        self._tst_srv = rospy.Service('~tests/battery/run', Trigger, self._tst)
        # test settings

    def _tst(self, _):
        success = True

        try:
            url = f"http://localhost/health/battery/info"
            data = requests.get(url).json()
            response = (
                f"version: {data['version']}\n"
                f"boot/pcb_version: {data['boot']['pcb_version']}\n"
            )

            url = f"http://localhost/health/battery"
            data = requests.get(url).json()
            response += f"battery/charging: {data['battery']['charging']}"
        except Exception as e:
            response = f"Error fetching battery info. {e}"
            success = False

        # Return the service response
        return TriggerResponse(success=success, message=response)

    def test_id(self) -> str:
        return f"Battery"

    def test_desc_preparation(self) -> str:
        return "N/A"

    def test_desc_expectation(self) -> str:
        return (
            "The battery firmware version should be at least `2.0.2'\n"
            "The PCB version should be at least `16'\n"
            "Within 2 seconds of (un)plugging the charging cable, the result should reflect whether the Duckiebot is being charged.\n"
            "Only mark the Success when the version examinations pass, and the charging state is indicated accurately."
        )

    def test_desc_running(self) -> str:
        return (
            "Run the test for multiple times, with and without the Duckiebot being charged;\n"
            "Each time after running, check the results against the Expected Outcomes.\n"
        )
    
    def test_desc_log_gather(self) -> str:
        return (
            "On your laptop, run the following command to save the logs.\n"
            "Replace the `[path/to/save]' to the directory path where you would like to save the logs.\n"
            "`docker -H [your_Duckiebot_hostname].local logs device-health > [path/to/save/]logs-db-iface.txt'\n"
            "Also on your laptop, run the following commands and save the logs in the terminal to text files.\n"
            "`dts duckiebot battery info [your_Duckiebot_hostname]'\n"
            "`dts duckiebot battery check_firmware [your_Duckiebot_hostname]"
        )

    def test_params(self) -> str:
        return f"[{self.test_id()}] N/A"

    def run_test(self) -> Optional[bool]:
        pass


class RobotRestAPInode(DTROS):
    def __init__(self):
        super(RobotRestAPInode, self).__init__(
            node_name="robot_http_api_node", node_type=NodeType.INFRASTRUCTURE, dt_ghost=True
        )

        self.hw_test_battery = HWTestBattery()
        self.hw_test_wifi = HWTestWifi()


def signal_handler(sig, frame):
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    ros_node = RobotRestAPInode()
    api = RobotRestAPI(debug=False)
    api.run(host="0.0.0.0")
