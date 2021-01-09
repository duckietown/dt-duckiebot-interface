#!/usr/bin/env python3
import time

import cv2
import copy
import rospy
import numpy as np

from PIL import Image
from typing import Any, Iterable
from threading import Semaphore

from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306

from cv_bridge import CvBridge
from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg

from duckietown.dtros import DTROS, NodeType, TopicType

from display_renderer import \
    REGION_FULL, \
    REGION_HEADER, \
    REGION_BODY, \
    REGION_FOOTER, \
    ALL_SCREENS, \
    AbsDisplayFragmentRenderer, \
    DisplayROI, \
    DisplayFragment


class DisplayNode(DTROS):
    _REGIONS = {
        DisplayFragmentMsg.REGION_FULL: REGION_FULL,
        DisplayFragmentMsg.REGION_HEADER: REGION_HEADER,
        DisplayFragmentMsg.REGION_BODY: REGION_BODY,
        DisplayFragmentMsg.REGION_FOOTER: REGION_FOOTER
    }

    def __init__(self):
        super(DisplayNode, self).__init__(
            node_name='display_driver_node',
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
        # screen selector
        self._screen = 0
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
            queue_size=10,
            buf_size='10M',
            dt_topic_type=TopicType.DRIVER,
            dt_help="Data to display on the display"
        )
        # create pager renderer
        self._pager_renderer = PagerFragmentRenderer()
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
            cw, ch = min(region.width - fx, roi.w), min(region.height - fy, roi.h)
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
                data=img, roi=roi, screen=msg.screen, z=msg.z,
                _ttl=msg.ttl, _time=msg.header.stamp.to_sec()
            )

    def _render(self, _):
        with self._fragments_lock:
            # remove expired fragments
            for region, fragments in self._fragments.items():
                for fragment_id in copy.copy(set(fragments.keys())):
                    fragment = fragments[fragment_id]
                    if fragment.ttl() <= 0:
                        self.logdebug(f"Fragment `{fragment_id}` on screen `{fragment.screen}`, "
                                      f"region `{region}` w/ TTL `{fragment.given_ttl}` "
                                      f"expired, remove!")
                        del self._fragments[region][fragment_id]
            # filter fragments by screen
            data = {
                region: [
                    fragment for fragment in fragments.values()
                    if fragment.screen in [ALL_SCREENS, self._screen]
                ] for region, fragments in self._fragments.items()
            }
        # sort fragments by z-index
        data = {
            region: sorted(fragments, key=lambda f: f.z)
            for region, fragments in data.items()
        }
        # clear buffer
        self._buffer.fill(0)
        # render fragments
        screens = {0}
        screens.add(self._screen)
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
                if fragment.screen != ALL_SCREENS:
                    screens.add(fragment.screen)
        # convert buffer to 1-byte pixel
        buf = Image.fromarray(self._buffer, mode='L')
        buf = buf.convert(mode='1')
        # display buffer
        with self._device_lock:
            try:
                self._display.display(buf)
            except BlockingIOError:
                pass
        # update pager for the next iteration
        self._pager_renderer.update(screens, self._screen)
        self._fragment_cb(self._pager_renderer.as_msg())

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
            try:
                self._display.display(buf)
            except BlockingIOError:
                pass


class PagerFragmentRenderer(AbsDisplayFragmentRenderer):

    SPACING_PX = 7
    UNSELECTED_SCREEN = np.array([
        [0,       0,      0,      0,      0],
        [0,       0,      0,      0,      0],
        [0,       0,      0,      0,      0],
        [0,     255,    255,    255,      0],
    ])
    SELECTED_SCREEN = np.array([
        [0,       0,      0,      0,      0],
        [255,   255,    255,    255,    255],
        [255,   255,    255,    255,    255],
        [255,   255,    255,    255,    255]
    ])

    def __init__(self):
        super(PagerFragmentRenderer, self).__init__(
            'pager',
            screen=ALL_SCREENS,
            region=REGION_FOOTER,
            roi=DisplayROI(0, 0, REGION_FOOTER.width, REGION_FOOTER.height)
        )
        self._screens = {0}
        self._screen = 0

    def update(self, screens: Iterable[int], screen: int):
        self._screens = screens
        self._screen = screen if screen in screens else 0

    def _render(self):
        ch, cw = self.shape
        _, sw = self.SELECTED_SCREEN.shape
        num_screens = len(self._screens)
        expected_w = num_screens * sw + (num_screens - 1) * self.SPACING_PX
        offset = int(np.floor((cw - expected_w) * 0.5))
        for screen in sorted(self._screens):
            icon = self.SELECTED_SCREEN if screen == self._screen else self.UNSELECTED_SCREEN
            self._buffer[:, offset:offset + sw] = icon
            offset += sw + self.SPACING_PX


if __name__ == '__main__':
    node = DisplayNode()
    rospy.spin()
