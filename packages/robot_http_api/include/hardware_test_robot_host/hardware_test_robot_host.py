import re
import subprocess
import requests
import netifaces as ni
import rospy

from dt_duckiebot_hardware_tests import HardwareTest, HardwareTestJsonParamType


class HardwareTestWifi(HardwareTest):
    def __init__(self, wifi_interface: str = "wlan0") -> None:
        super().__init__(service_identifier="tests/wifi")

        # attr
        self._interface = wifi_interface

    def test_id(self) -> str:
        return "USB Wifi Dongle"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(
            [
                "Make sure the USB Wifi dongle is plugged in to your Duckiebot, and it is blinking.",
            ]
        )

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                "The IP address of the <code>wlan0</code> network interface should be displayed below.",
            ]
        )

    def test_description_log_gather(self) -> str:
        return self.html_util_ul(
            [
                "On your laptop, run the following command to save the logs.",
                "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
                "<code>ssh duckie@[ROBOT_NAME].local ifconfig > [path/to/save/]logs-network-ifconfig.txt</code>",
                "(You might need to provide the password to your Duckiebot when prompted.)",
            ]
        )

    def _get_ipv4_addr(self) -> str:
        addrs = ni.ifaddresses(self._interface)
        if ni.AF_INET in addrs:
            return addrs[ni.AF_INET][0]["addr"]
        return "None"

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
        success = True
        response = ""

        try:
            response = self._get_ipv4_addr()
        except Exception as e:
            rospy.logerr(f"[{self.test_id()}] Experienced error: {e}")
            success = False

        return self.format_response_object(
            success=success,
            lst_blocks=[
                self.format_obj(
                    key=f"Getting the IP of {self._interface}:",
                    value_type=HardwareTestJsonParamType.STRING,
                    value=response,
                ),
            ],
        )


class HardwareTestBattery(HardwareTest):
    def __init__(self) -> None:
        super().__init__(service_identifier="tests/battery")

    def test_id(self) -> str:
        return "Battery"

    def test_description_preparation(self) -> str:
        return self.html_util_ul(["Place your Duckiebot near the charging cable."])

    def test_description_expectation(self) -> str:
        return self.html_util_ul(
            [
                "The battery firmware version should be at least <code>2.0.2</code>.",
                "The PCB version should be at least <code>16</code>.",
                "The test result should reflect the accurate charging status, within 2 seconds of a change.",
                "Only mark the Success when the version examinations pass, and the charging states are indicated correctly.",
            ]
        )

    def test_description_running(self) -> str:
        return self.html_util_ul(
            [
                "Run the test for multiple times, with and without the Duckiebot being charged.",
                "Check the results against the Expected Outcomes each time.",
                "Please click the <strong>Run the Test</strong> button below to run the tests.",
            ]
        )

    def test_description_log_gather(self) -> str:
        return self.html_util_ul(
            [
                "On your laptop, run the following command to save the logs.",
                "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
                "<code>docker -H [ROBOT_NAME].local logs device-health > [path/to/save/]logs-db-device-health.txt</code>",
                "Also on your laptop, run the following commands and save the logs in the terminal to text files.",
                "<code>dts duckiebot battery info [ROBOT_NAME]</code>",
                "<code>dts duckiebot battery check_firmware [ROBOT_NAME]</code>",
            ]
        )

    def cb_run_test(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
        success = True
        response = ""

        try:
            # check versions
            url = f"http://localhost/health/battery/info"
            data_info = requests.get(url).json()
            # check charging status
            url = f"http://localhost/health/battery"
            data = requests.get(url).json()
            # format response

            response = self.html_util_ul(
                [
                    f"version: <code>{data_info['version']}</code>",
                    f"boot/pcb_version: <code>{data_info['boot']['pcb_version']}</code>",
                    f"battery/charging: <strong>{data['battery']['charging']}</strong>",
                ]
            )
        except Exception as e:
            rospy.logerr(f"[{self.test_id()}] Experienced error: {e}")
            success = False

        return self.format_response_object(
            success=success,
            lst_blocks=[
                self.format_obj(
                    key="Battery status",
                    value_type=HardwareTestJsonParamType.HTML,
                    value=response,
                ),
            ],
        )
