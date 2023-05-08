#!/usr/bin/env python3

import dataclasses
import time
from typing import Optional

import numpy as np
import rospy
import yaml
from display_renderer import (
    DisplayROI,
    PAGE_TOF,
    REGION_BODY,
    MonoImageFragmentRenderer,
)
from display_renderer.text import monospace_screen
from dt_class_utils import DTReminder
from dt_vl53l0x import VL53L0X, Vl53l0xAccuracyMode
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import DisplayFragment
from sensor_msgs.msg import Range
from std_msgs.msg import Header

from abc import ABC, abstractmethod
from typing import Optional, Dict, List, Union
from std_srvs.srv import Trigger, TriggerResponse
from enum import Enum
import json


class HWTestJsonParamType(Enum):
    STRING = 'string'
    BASE64 = 'base64'
    HTML = 'html'
    OBJECT = 'object'
    STREAM = 'stream'


class EnumJSONEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, Enum):
            return obj.value
        return super().default(obj)


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
        return self.html_util_ul(["Click on the <strong>Run the test</strong> button below."])

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
        return [
            self.format_obj(
                key="Preparation",
                value_type=HWTestJsonParamType.HTML,
                value=self.test_desc_preparation(),
            ),
            self.format_obj("Expected Outcomes", HWTestJsonParamType.HTML, self.test_desc_expectation()),
            self.format_obj("How to run", HWTestJsonParamType.HTML, self.test_desc_running()),
            self.format_obj("Logs Gathering (in case of errors)", HWTestJsonParamType.HTML, self.test_desc_log_gather()),
        ]

    @staticmethod
    def format_obj(key: str, value_type: "HWTestJsonParamType", value: str):
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
        ret_obj = {
            "type": HWTestJsonParamType.STREAM,
            "parameters": []
        }
        for block in lst_blocks:
            ret_obj["parameters"].append(block)

        ret_obj["parameters"].append(HWTest.format_obj(
            key="test_topic_name",
            value_type="str",
            value=test_topic_name,
        ))

        ret_obj["parameters"].append(HWTest.format_obj(
            key="test_topic_type",
            value_type="str",
            value=test_topic_type,
        ))

        return TriggerResponse(
            success=success,
            message=json.dumps(ret_obj, cls=EnumJSONEncoder),
        )

    @staticmethod
    def format_response_object(success: bool, lst_blocks):
        ret_obj = {
            "type": HWTestJsonParamType.OBJECT,
            "parameters": []
        }
        for block in lst_blocks:
            ret_obj["parameters"].append(block)

        return TriggerResponse(
            success=success,
            message=json.dumps(ret_obj, cls=EnumJSONEncoder),
        )

    @staticmethod
    def html_util_ul(lst_items: List[str]) -> str:
        ret = ["<ul>"]
        for item in lst_items:
            ret.append("<li>" + item + "</li>")
        ret.append("</ul>")
        return "".join(ret)

    def srv_cb_tst_desc(self, _):
        return self.format_response_object(
            success=True,
            lst_blocks=self.test_desc(),
        )


class HWTestToF(HWTest):
    def __init__(self, sensor_name: str, accuracy: "ToFAccuracy") -> None:
        super().__init__()
        self._desc_tst_srv = rospy.Service('~test/desc', Trigger, self.srv_cb_tst_desc)
        self._tst_srv = rospy.Service('~test/run', Trigger, self._tst)
        # test settings
        # attr
        self._sensor_name = sensor_name
        self._accuracy = accuracy

    def _tst(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")

        instructions = self.html_util_ul([
            f"Now move your hand in front of the <strong>{self._sensor_name}</strong> Time-of-Flight sensor at different distances.",
            "Once your decide the test has passed/failed, you may mark the decision, and close this modal.",
        ])

        # Return the service response
        return self.format_response_stream(
            success=True,  # does not matter here
            test_topic_name=f"{self._sensor_name}_tof_driver_node/range",
            test_topic_type="sensor_msgs/Range",
            lst_blocks=[
                self.format_obj(
                    key="Instructions",
                    value_type=HWTestJsonParamType.HTML,
                    value=instructions,
                ),
            ],
        )

    def test_id(self) -> str:
        return f"Time-of-Flight ({self._sensor_name})"

    def test_desc_preparation(self) -> str:
        return self.html_util_ul([
            "Put your Duckiebot in its normal orientation.",
            "Place it near you, with its camera facing an empty space.",
        ])

    def test_desc_expectation(self) -> str:
        return self.html_util_ul([
            "Once your start the test, a <strong>Range:</strong> field will appear below.",
            f"When you move your hand closer and farther to the {self._sensor_name} ToF, the range reading should change accordingly, i.e. moving closer leads to a smaller value, and farther with a greater value.",
            f"The effective range of the sensor is <strong>from {self._accuracy.min_range}m to {self._accuracy.max_range}m</strong>. The data below should show (in red color) <em>Out of range</em>."
        ])

    def test_desc_log_gather(self) -> str:
        return self.html_util_ul([
            "On your laptop, run the following command to save the logs.",
            "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
            "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
        ])

    def test_params(self) -> str:
        return f"[{self.test_id()}] N/A"


@dataclasses.dataclass
class ToFAccuracy:
    mode: Vl53l0xAccuracyMode
    timing_budget: float
    max_range: float
    # the following are taken from the sensor's datasheet
    min_range: float = 0.05
    fov: float = np.deg2rad(25)

    @staticmethod
    def from_string(mode: str):
        ms = 1 / 1000
        return {
            "GOOD": ToFAccuracy(Vl53l0xAccuracyMode.GOOD, 33 * ms, 1.2),
            "BETTER": ToFAccuracy(Vl53l0xAccuracyMode.BETTER, 66 * ms, 1.2),
            "BEST": ToFAccuracy(Vl53l0xAccuracyMode.BEST, 200 * ms, 1.2),
            "LONG_RANGE": ToFAccuracy(Vl53l0xAccuracyMode.LONG_RANGE, 33 * ms, 2.0),
            "HIGH_SPEED": ToFAccuracy(Vl53l0xAccuracyMode.HIGH_SPEED, 20 * ms, 1.2),
        }[mode]


class ToFNode(DTROS):
    def __init__(self):
        super(ToFNode, self).__init__(node_name="tof_node", node_type=NodeType.DRIVER)
        # get parameters
        self._veh = rospy.get_param("~veh")
        self._i2c_connectors = rospy.get_param("~connectors", {})
        self._sensor_name = rospy.get_param("~sensor_name")
        self._frequency = int(max(1, rospy.get_param("~frequency", 10)))
        self._mode = rospy.get_param("~mode", "BETTER")
        self._display_fragment_frequency = rospy.get_param("~display_fragment_frequency", 4)
        self._accuracy = ToFAccuracy.from_string(self._mode)
        # create a VL53L0X sensor handler
        self._sensor: Optional[VL53L0X] = self._find_sensor()
        if not self._sensor:
            conns: str = yaml.safe_dump(self._i2c_connectors, indent=2, sort_keys=True)
            self.logerr(f"No VL53L0X device found. These connectors were tested:\n{conns}\n")
            exit(1)
        # create publisher
        self._pub = rospy.Publisher(
            "~range",
            Range,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="The distance to the closest object detected by the sensor",
        )
        self._display_pub = rospy.Publisher(
            "~fragments",
            DisplayFragment,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION,
            dt_help="Fragments to display on the display",
        )
        # hardware test
        self._hw_test = HWTestToF(self._sensor_name, self._accuracy)

        # create screen renderer
        self._renderer = ToFSensorFragmentRenderer(self._sensor_name, self._accuracy)
        # check frequency
        max_frequency = min(self._frequency, int(1.0 / self._accuracy.timing_budget))
        if self._frequency > max_frequency:
            self.logwarn(
                f"Frequency of {self._frequency}Hz not supported. The selected mode "
                f"{self._mode} has a timing budget of {self._accuracy.timing_budget}s, "
                f"which yields a maximum frequency of {max_frequency}Hz."
            )
            self._frequency = max_frequency
        self.loginfo(f"Frequency set to {self._frequency}Hz.")
        # create timers
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / max_frequency), self._timer_cb)
        self._fragment_reminder = DTReminder(frequency=self._display_fragment_frequency)

    def _find_sensor(self) -> Optional[VL53L0X]:
        for connector in self._i2c_connectors:
            conn: str = "[bus:{bus}](0x{address:02X})".format(**connector)
            self.loginfo(f"Trying to open device on connector {conn}")
            sensor = VL53L0X(i2c_bus=connector["bus"], i2c_address=connector["address"])
            try:
                sensor.open()
            except FileNotFoundError:
                # i2c BUS not found
                self.logwarn(f"No devices found on connector {conn}, the bus does NOT exist")
                continue
            sensor.start_ranging(self._accuracy.mode)
            time.sleep(1)
            if sensor.get_distance() < 0:
                self.logwarn(f"No devices found on connector {conn}, but the bus exists")
                continue
            self.loginfo(f"Device found on connector {conn}")
            return sensor

    def _timer_cb(self, _):
        # detect range
        distance_mm = self._sensor.get_distance()
        # pack observation into a message
        msg = Range(
            header=Header(stamp=rospy.Time.now(), frame_id=f"{self._veh}/tof/{self._sensor_name}"),
            radiation_type=Range.INFRARED,
            field_of_view=self._accuracy.fov,
            min_range=self._accuracy.min_range,
            max_range=self._accuracy.max_range,
            range=distance_mm / 1000,
        )
        # publish
        self._pub.publish(msg)
        # publish display rendering (if it is a good time to do so)
        if self._fragment_reminder.is_time():
            self._renderer.update(distance_mm)
            msg = self._renderer.as_msg()
            self._display_pub.publish(msg)

    def on_shutdown(self):
        # noinspection PyBroadException
        try:
            self._sensor.stop_ranging()
        except BaseException:
            pass


class ToFSensorFragmentRenderer(MonoImageFragmentRenderer):
    def __init__(self, name: str, accuracy: ToFAccuracy):
        super(ToFSensorFragmentRenderer, self).__init__(
            f"__tof_{name}__",
            page=PAGE_TOF,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
        )
        self._name = name
        self._accuracy = accuracy
        name = self._name.replace("_", " ").title()
        self._title_h = 12
        self._title = monospace_screen((self._title_h, self.roi.w), f"ToF / {name}:", scale="vfill")

    def update(self, measurement_mm: float):
        pretty_measurement = (
            f" {(measurement_mm / 10):.1f}cm "
            if (measurement_mm / 1000) < self._accuracy.max_range
            else "Out-Of-Range"
        )
        reading = monospace_screen(
            (self.roi.h - self._title_h, self.roi.w), pretty_measurement, scale="hfill", align="center"
        )
        self.data[: self._title_h, :] = self._title
        self.data[self._title_h:, :] = reading


if __name__ == "__main__":
    node = ToFNode()
    rospy.spin()
