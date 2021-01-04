#!/usr/bin/env python3
import abc
import time
from threading import Semaphore
from typing import Any

import dataclasses
import rospy
import cv2
from PIL import Image
import numpy as np
from cv_bridge import CvBridge

from std_msgs.msg import Header
from sensor_msgs.msg import RegionOfInterest
from duckietown_msgs.msg import DisplayFragment


from duckietown.dtros import DTROS, NodeType, TopicType



class HealthDisplayRendererNode(DTROS):

    def __init__(self):
        super(HealthDisplayRendererNode, self).__init__(
            node_name='health_renderer_node',
            node_type=NodeType.VISUALIZATION
        )
        # get parameters
        self._assets_dir = rospy.get_param('~assets_dir')
        self._frequency = rospy.get_param('~frequency')
        # create a cv bridge instance
        self._bridge = CvBridge()
        # create publisher
        self._pub = rospy.Publisher(
            "~fragments",
            DisplayFragment,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION,
            dt_help="Fragments to display on the display"
        )
        # create loop
        self._timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self._frequency), self._publish)

    def _publish(self, _):
        img = np.zeros((10, 128), dtype=np.uint8)
        img[0:3, 14:114] = 255
        msg = DisplayFragment(
            header=Header(stamp=rospy.Time.now()),
            id="battery_indicator",
            region=DisplayFragment.REGION_BODY,
            data=self._bridge.cv2_to_imgmsg(img, encoding="mono8"),
            # location=RegionOfInterest(
            #     x_offset=0, y_offset=0, width=128, height=10
            # ),
            ttl=10
        )
        self._pub.publish(msg)




if __name__ == '__main__':
    node = HealthDisplayRendererNode()
    rospy.spin()
