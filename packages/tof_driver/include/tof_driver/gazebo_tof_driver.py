# !/usr/bin/env python3
from typing import Optional

import rospy
from sensor_msgs.msg import LaserScan
from tof_driver.tof_driver_abs import ToFDriverAbs, ToFAccuracy


class GazeboToFDriver(ToFDriverAbs):

    def __init__(self, name: str, accuracy: ToFAccuracy, *_, **__):
        super(GazeboToFDriver, self).__init__(name, accuracy)
        self._lidar_topic = '/gazebo/lidar_node_virtual/scan' # We could retrieve this from rosparam that we set for the ros1 bridge
        self._range = None

    def setup(self):
        # Create a subscriber to the laser scan topic
        self._scan_subscriber = rospy.Subscriber(self._lidar_topic, LaserScan, self._process_scan)
        return

    def start(self):
        rospy.loginfo(f"Starting {self._name} driver")
        rospy.loginfo(f"Subscribing to {self._lidar_topic}")

    def _process_scan(self, msg: LaserScan):
        # Handle the case where msg.ranges[0] is inf by clipping it to 2m
        # convert [m] -> [mm]
        self._range = max(min(msg.ranges[0],self._accuracy.max_range), self._accuracy.min_range) * 1000.0

    def get_distance(self) -> Optional[float]:
        if self._range is None:
            return -1
        return self._range

    def stop(self):
        self._range = None
        rospy.loginfo(f"Stopping {self._name} driver")
        self._scan_subscriber.unregister()
        return

