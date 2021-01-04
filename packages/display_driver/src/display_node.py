#!/usr/bin/env python3
import time
from threading import Semaphore
from typing import Any

import dataclasses
import rospy
import cv2
from PIL import Image
import numpy as np
from cv_bridge import CvBridge
from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306

from duckietown.dtros import DTROS, NodeType, TopicType


@dataclasses.dataclass
class DisplayRegion:
    name: str
    width: int
    height: int
    x: int = 0
    y: int = 0


@dataclasses.dataclass
class DisplayROI:
    x: int
    y: int
    w: int
    h: int

    @staticmethod
    def from_sensor_msgs_ROI(roi):
        if roi.width + roi.height > 0:
            return DisplayROI(roi.x_offset, roi.y_offset, roi.width, roi.height)
        return None


@dataclasses.dataclass
class DisplayFragment:
    data: np.ndarray
    roi: DisplayROI
    z: int
    _time: float
    _ttl: int

    def ttl(self):
        if self._ttl < 0:
            # infinite ttl
            return 1
        elapsed = time.time() - self._time
        ttl = self._ttl - elapsed
        return ttl


class DisplayNode(DTROS):
    _REGIONS = {
        DisplayFragmentMsg.REGION_FULL: DisplayRegion("*", 128, 64),
        DisplayFragmentMsg.REGION_HEADER: DisplayRegion('header', 128, 16),
        DisplayFragmentMsg.REGION_BODY: DisplayRegion('body', 128, 48, y=16),
    }

    def __init__(self):
        super(DisplayNode, self).__init__(
            node_name='display_node',
            node_type=NodeType.DRIVER
        )
        # get parameters
        self._veh = rospy.get_param('~veh')
        self._i2c_bus = rospy.get_param('~bus', 1)
        self._i2c_address = rospy.get_param('~address', 0x3C)
        self._frequency = rospy.get_param('~frequency', 1)
        # create a display handler
        serial = i2c(port=self._i2c_bus, address=self._i2c_address)
        self._display = ssd1306(serial)
        # create a cv bridge instance
        self._bridge = CvBridge()
        # create buffers
        self._fragments = {
            k: dict() for k in self._REGIONS
        }
        self._buffer = np.zeros((
            self._REGIONS[DisplayFragmentMsg.REGION_FULL].height,
            self._REGIONS[DisplayFragmentMsg.REGION_FULL].width),
            dtype=np.uint8
        )
        self._fragments_lock = Semaphore(1)
        self._device_lock = Semaphore(1)
        # create subscribers
        self._sub = rospy.Subscriber(
            "~fragments",
            DisplayFragmentMsg,
            self._fragment_cb,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="Data to display on the display"
        )
        # create rendering loop
        self._timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self._frequency), self._render)

    def _fragment_cb(self, msg: Any):
        region = self._REGIONS[msg.region]
        # convert image to greyscale
        img = self._bridge.imgmsg_to_cv2(msg.data, "mono8")
        # parse ROI
        roi = DisplayROI.from_sensor_msgs_ROI(msg.location)
        if roi is None:
            # no ROI was specified
            # the fragment will be used as is if it fits the region, resized otherwise
            rh, rw = region.height, region.width
            fh, fw = img.shape
            # does it fit?
            if fw > rw or fh > rh:
                img = cv2.resize(img, (rw, rh), interpolation=cv2.INTER_NEAREST)
                fh, fw = rh, rw
            roi = DisplayROI(x=0, y=0, w=fw, h=fh)
        else:
            # validate ROI
            fh, fw = img.shape
            # - find biggest offsets achievable
            fx, fy = min(region.width, roi.x), min(region.height, roi.y)
            # - find biggest canvas size achievable
            cw, ch = min(region.width, roi.w) - fx, min(region.height, roi.h) - fy
            # does it fit?
            if fw > cw or fh > ch:
                img = cv2.resize(img, (cw, ch), interpolation=cv2.INTER_NEAREST)
                fh, fw = ch, cw
            # update ROI
            roi = DisplayROI(x=fx, y=fy, w=fw, h=fh)

        # threshold image at mid-range
        img = (img > 125).astype(np.uint8) * 255
        # list fragment for rendering
        with self._fragments_lock:
            self._fragments[msg.region][msg.id] = DisplayFragment(
                data=img, roi=roi, z=msg.z, _ttl=msg.ttl, _time=msg.header.stamp.to_sec()
            )

    def _render(self, _):
        with self._fragments_lock:
            # remove expired fragments
            self._fragments = {
                region: {
                    fragment_id: fragment
                    for fragment_id, fragment in fragments.items() if fragment.ttl() > 0
                } for region, fragments in self._fragments.items()
            }
            # sort fragments by z-index
            data = {
                region: sorted(fragments.values(), key=lambda f: f.z)
                for region, fragments in self._fragments.items()
            }
        # clear buffer
        self._buffer.fill(0)
        # render regions
        for region_id, fragments in data.items():
            region = self._REGIONS[region_id]
            # render fragments
            for fragment in fragments:
                fx, fy = fragment.roi.x, fragment.roi.y
                fw, fh = fragment.roi.w, fragment.roi.h
                # move fragment to the right region
                fx += region.x
                fy += region.y
                # update buffer
                self._buffer[fy:fy + fh, fx:fx + fw] = fragment.data
        # convert buffer to 1-byte pixel
        buf = Image.fromarray(self._buffer, mode='L')
        buf = buf.convert(mode='1')
        # display buffer
        with self._device_lock:
            self._display.display(buf)

    def on_shutdown(self):
        self.loginfo("Clearing buffer...")
        # stop rendering job
        self._timer.shutdown()
        # clear buffer
        self._buffer.fill(0)
        # render nothing
        buf = Image.fromarray(self._buffer, mode='1')
        self.loginfo("Clearing display...")
        with self._device_lock:
            self._display.display(buf)


if __name__ == '__main__':
    node = DisplayNode()
    rospy.spin()