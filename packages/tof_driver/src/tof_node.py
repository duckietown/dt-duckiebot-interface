#!/usr/bin/env python3

import dataclasses
import time
from typing import Optional

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
from dt_vl53l0x import VL53L0X
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import DisplayFragment
from sensor_msgs.msg import Range
from std_msgs.msg import Header

from tof_accuracy import ToFAccuracy
from hardware_test_tof import HardwareTestToF


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
        # user hardware test
        self._hardware_test = HardwareTestToF(self._sensor_name, self._accuracy)

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
