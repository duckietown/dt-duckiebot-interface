#!/usr/bin/env python3

import rospy
import requests
from typing import Dict

from dt_robot_utils import get_robot_configuration
from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg

from display_renderer import (
    PAGE_ROBOT_INFO,
    REGION_BODY,
    DisplayROI,
    TextFragmentRenderer,
)

from duckietown.dtros import DTROS, NodeType, TopicType


class RobotInfoRendererNode(DTROS):
    def __init__(self):
        super(RobotInfoRendererNode, self).__init__(
            node_name="robot_info_renderer_node", node_type=NodeType.VISUALIZATION
        )
        # get parameters
        self._veh = rospy.get_param("~veh")
        self._frequency = rospy.get_param("~frequency")
        # create publisher
        self._pub = rospy.Publisher(
            "~fragments",
            DisplayFragmentMsg,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION,
            dt_help="Fragments to display on the display",
        )
        # create renderers
        self._renderer = RobotInfoRenderer()
        # create loop
        self._timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self._frequency), self._beat)

    def _beat(self, _):
        robot_info_str = self._fetch()
        self._renderer.update(robot_info_str)
        self._pub.publish(self._renderer.as_msg())

    def _fetch(self) -> str:
        health_api_url = f"http://{self._veh}.local/health/"
        # noinspection PyBroadException
        try:
            health_data = requests.get(health_api_url).json()
        except BaseException:
            return

        sw_date = health_data["software"]["date"]
        sw_version = health_data["software"]["version"]
        firmware = f"{sw_date['day']}/{sw_date['month']}/{sw_date['year']} ({sw_version})"

        return self._fmt({
            "Name": self._veh,
            "Model": get_robot_configuration().name,
            "FW": firmware,
        })

    def _fmt(self, disp_data: Dict[str, str]) -> str:
        # align keys to the left, align colons, and values to the right
        max_key_length = max(len(key) for key in disp_data.keys())
        max_value_length = max(len(str(value)) for value in disp_data.values())
        output = ""
        for key, value in disp_data.items():
            output += "{:<{}}: {:>{}}\n".format(key, max_key_length, str(value), max_value_length)
        return output


class RobotInfoRenderer(TextFragmentRenderer):
    def __init__(self):
        super(RobotInfoRenderer, self).__init__(
            name=f"__robot_info__",
            page=PAGE_ROBOT_INFO,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
            scale="fill",
            ttl=20,
        )


if __name__ == "__main__":
    node = RobotInfoRendererNode()
    rospy.spin()
