"""
User hardware test template.

The user hardware tests are embedded in the Robot Dashboard,
on the Components page. Each test is tied to a hardware driver ROS package,
in the dt-duckiebuto-interface repository. The purpose of this is to help users
verify the basic behaviors of the hardware.

Here're the steps to write a new test:
1.  In dt-duckiebot-interface, in a hardware driver ROS package, a test is
    defined deriving from this. A ROS service is defined how the test is triggered.
2.  In dt-device-health, the test service is registered with each "component"
3.  In the Dashboard, the components are rendered by querying dt-device-health,
    and rendering of test responses is written there.
"""

import json
import rospy

from abc import ABC, abstractmethod
from enum import Enum
from typing import List

from std_srvs.srv import Trigger, TriggerResponse


class HardwareTestJsonParamType(Enum):
    """Type constants, so receiving side knows how to parse/use the values"""

    # basic string
    STRING = "string"
    # base64 encode image
    BASE64 = "base64"
    # a html block (as a string)
    HTML = "html"
    # ROS Topic related info, i.e. name and type. for the "stream" option below
    TOPIC_INFO = "topic_info"

    # --- Highest level response types ---
    # results of a completed test
    OBJECT = "object"
    # instructions to connect to a live stream from a topic
    STREAM = "stream"


class EnumJSONEncoder(json.JSONEncoder):
    """Used to encode the HardwareTestJsonParamType(Enum)"""

    def default(self, obj):
        if isinstance(obj, Enum):
            return obj.value
        return super().default(obj)


class HardwareTest(ABC):
    def __init__(self, service_identifier: str = "test"):
        """Base class for User Hardware Tests

        Args:
            service_identifier (str, optional): The resulting test services are [node_prefix]/[identifier]/[description|run]. Defaults to "test".
        """
        # test services
        self.description_srv = rospy.Service(
            f"~{service_identifier}/description", Trigger, self.cb_description
        )
        self.test_srv = rospy.Service(
            f"~{service_identifier}/run", Trigger, self.cb_run_test
        )

    @abstractmethod
    def test_id(self) -> str:
        """Short name, used to report start and end of test"""
        pass

    @abstractmethod
    def test_description_preparation(self) -> str:
        """Preparation before running. E.g. put the DB upside down"""
        pass

    def test_description_running(self) -> str:
        """Actual steps to run the test"""
        # default: just click the "Run test" button
        return self.html_util_ul(
            ["Click on the <strong>Run the test</strong> button below."]
        )

    @abstractmethod
    def test_description_expectation(self) -> str:
        """Expected outcome(s) and/or how to determine if a test was successful"""
        pass

    def test_description_log_gather(self) -> str:
        """How to gather logs before reporting"""
        return self.html_util_ul(
            [
                "On your laptop, run the following command to save the logs.",
                "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
                "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
            ]
        )

    def test_description(self) -> str:
        """Test descriptions"""
        return [
            self.format_obj(
                key="Preparation",
                value_type=HardwareTestJsonParamType.HTML,
                value=self.test_description_preparation(),
            ),
            self.format_obj(
                "Expected Outcomes",
                HardwareTestJsonParamType.HTML,
                self.test_description_expectation(),
            ),
            self.format_obj(
                "How to run",
                HardwareTestJsonParamType.HTML,
                self.test_description_running(),
            ),
            self.format_obj(
                "Logs Gathering (in case of errors)",
                HardwareTestJsonParamType.HTML,
                self.test_description_log_gather(),
            ),
        ]

    def cb_description(self, _):
        """The test description service response"""
        return self.format_response_object(
            success=True,
            lst_blocks=self.test_description(),
        )

    @abstractmethod
    def cb_run_test(self, _):
        """Actually running the test"""
        pass

    @staticmethod
    def format_obj(key: str, value_type: "HardwareTestJsonParamType", value: str):
        return {
            "key": key,
            "type": value_type,
            "value": value,
        }

    @staticmethod
    def format_response_stream(
        success: bool,
        test_topic_name: str,
        test_topic_type: str,
        lst_blocks,
    ):
        ret_obj = {"type": HardwareTestJsonParamType.STREAM, "parameters": []}
        for block in lst_blocks:
            ret_obj["parameters"].append(block)

        ret_obj["parameters"].append(
            HardwareTest.format_obj(
                key="test_topic_name",
                value_type=HardwareTestJsonParamType.TOPIC_INFO,
                value=test_topic_name,
            )
        )

        ret_obj["parameters"].append(
            HardwareTest.format_obj(
                key="test_topic_type",
                value_type=HardwareTestJsonParamType.TOPIC_INFO,
                value=test_topic_type,
            )
        )

        return TriggerResponse(
            success=success,
            message=json.dumps(ret_obj, cls=EnumJSONEncoder),
        )

    @staticmethod
    def format_response_object(success: bool, lst_blocks):
        ret_obj = {"type": HardwareTestJsonParamType.OBJECT, "parameters": []}
        for block in lst_blocks:
            ret_obj["parameters"].append(block)

        return TriggerResponse(
            success=success,
            message=json.dumps(ret_obj, cls=EnumJSONEncoder),
        )

    @staticmethod
    def html_util_ul(lst_items: List[str]) -> str:
        """Helper function to add a list of html code to an unordered list"""
        ret = ["<ul>"]
        for item in lst_items:
            ret.append("<li>" + item + "</li>")
        ret.append("</ul>")
        return "".join(ret)
