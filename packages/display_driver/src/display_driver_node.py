#!/usr/bin/env python3

import copy
import rospy
import numpy as np

from PIL import Image
from typing import Any, Iterable
from threading import Semaphore

from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306

from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg
from duckietown_msgs.msg import ButtonEvent as ButtonEventMsg

from dt_class_utils import DTReminder
from duckietown.dtros import DTROS, NodeType, TopicType

from display_renderer import (
    REGION_FULL,
    REGION_HEADER,
    REGION_BODY,
    REGION_FOOTER,
    ALL_PAGES,
    PAGE_HOME,
    PAGE_SHUTDOWN,
    AbsDisplayFragmentRenderer,
    DisplayROI,
    DisplayFragment,
)

from duckietown.utils.image.pil import np_to_pil, pil_to_np
from duckietown.utils.image.ros import imgmsg_to_mono8


class DisplayNode(DTROS):
    _REGIONS = {
        DisplayFragmentMsg.REGION_FULL: REGION_FULL,
        DisplayFragmentMsg.REGION_HEADER: REGION_HEADER,
        DisplayFragmentMsg.REGION_BODY: REGION_BODY,
        DisplayFragmentMsg.REGION_FOOTER: REGION_FOOTER
    }
    _MAX_FREQUENCY_HZ = 5

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
        # page selector
        self._page = PAGE_HOME
        self._pages = {PAGE_HOME}
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
        self._fragments_sub = rospy.Subscriber(
            "~fragments",
            DisplayFragmentMsg,
            self._fragment_cb,
            queue_size=10,
            buf_size='4M',
            dt_topic_type=TopicType.DRIVER,
            dt_help="Data to display on the display"
        )
        self._button_sub = rospy.Subscriber(
            "~button",
            ButtonEventMsg,
            self._button_event_cb,
            queue_size=1,
            dt_help="Button event"
        )
        # create internal renderers
        self._pager_renderer = PagerFragmentRenderer()
        # create rendering loop
        self._timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self._frequency), self._render)
        self._reminder = DTReminder(frequency=self._MAX_FREQUENCY_HZ)

    def _button_event_cb(self, msg: Any):
        if msg.event == ButtonEventMsg.EVENT_SINGLE_CLICK:
            with self._fragments_lock:
                pages = sorted(self._pages)
                # move to the next page
                try:
                    i = pages.index(self._page)
                    i = pages[(i + 1) % len(pages)]
                except ValueError:
                    i = 0
                # select page
                self._page = i

        if msg.event == ButtonEventMsg.EVENT_HELD_3SEC:
            with self._fragments_lock:
                self._page = PAGE_SHUTDOWN
                    

    def _fragment_cb(self, msg: Any):
        region = self._REGIONS[msg.region]
        # convert image to greyscale
        img = imgmsg_to_mono8(msg.data)
        # parse ROI
        roi = DisplayROI.from_sensor_msgs_ROI(msg.location)
        if roi is None:
            # no ROI was specified
            # the fragment will be used as is if it fits the region, resized otherwise
            rh, rw = region.height, region.width
            fh, fw = img.shape
            # does it fit?
            if fw > rw or fh > rh:
                img = np_to_pil(img, mode="L")
                img = img.resize((rw, rh), resample=Image.NEAREST)
                img = pil_to_np(img)
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
                img = np_to_pil(img, mode="L")
                img = img.resize((cw, ch), resample=Image.NEAREST)
                img = pil_to_np(img)
                fh, fw = ch, cw
            # update ROI
            roi = DisplayROI(x=fx, y=fy, w=fw, h=fh)

        # threshold image at mid-range
        img = (img > 125).astype(np.uint8) * 255
        # list fragment for rendering
        with self._fragments_lock:
            self._fragments[msg.region][msg.id] = DisplayFragment(
                data=img, roi=roi, page=msg.page, z=msg.z,
                _ttl=msg.ttl, _time=msg.header.stamp.to_sec()
            )
        # force refresh if this fragment is on the current page
        if msg.page == self._page:
            self._render(None)

    def _render(self, _):
        # use a reminder object to control the maximum frequency
        if not self._reminder.is_time():
            return
        # make sure we are not shutdown
        if self.is_shutdown:
            return
        # ---
        with self._fragments_lock:
            # clean pages
            self._pages = {PAGE_HOME}
            # remove expired fragments and annotate how many pages we need
            for region, fragments in self._fragments.items():
                for fragment_id in copy.copy(set(fragments.keys())):
                    fragment = fragments[fragment_id]
                    if fragment.ttl() <= 0:
                        self.logdebug(f"Fragment `{fragment_id}` on page `{fragment.page}`, "
                                      f"region `{region}` w/ TTL `{fragment.given_ttl}` "
                                      f"expired, remove!")
                        del self._fragments[region][fragment_id]
                    if fragment.page != ALL_PAGES:
                        self._pages.add(fragment.page)
            # sanitize page selector
            if self._page not in self._pages:
                # go back to home
                self._page = PAGE_HOME
            # filter fragments by page
            data = {
                region: [
                    fragment for fragment in fragments.values()
                    if fragment.page in [ALL_PAGES, self._page]
                ] for region, fragments in self._fragments.items()
            }
        # add pager fragment
        self._pager_renderer.update(self._pages, self._page)
        if self._pager_renderer.page in [ALL_PAGES, self._page]:
            data[self._pager_renderer.region.id].append(self._pager_renderer.as_fragment())
        # sort fragments by z-index
        data = {
            region: sorted(fragments, key=lambda f: f.z)
            for region, fragments in data.items()
        }
        # clear buffer
        self._buffer.fill(0)
        # render fragments
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
        # convert 1-byte pixel to 1-bit pixel
        buf = buf.convert(mode='1')
        # display buffer
        with self._device_lock:
            try:
                self._display.display(buf)
            except BlockingIOError:
                pass

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
    UNSELECTED_PAGE_ICON = np.array([
        [0,       0,      0,      0,      0],
        [0,       0,      0,      0,      0],
        [0,       0,      0,      0,      0],
        [0,     255,    255,    255,      0],
    ])
    SELECTED_PAGE_ICON = np.array([
        [0,       0,      0,      0,      0],
        [255,   255,    255,    255,    255],
        [255,   255,    255,    255,    255],
        [255,   255,    255,    255,    255]
    ])

    def __init__(self):
        super(PagerFragmentRenderer, self).__init__(
            'pager',
            page=ALL_PAGES,
            region=REGION_FOOTER,
            roi=DisplayROI(0, 0, REGION_FOOTER.width, REGION_FOOTER.height)
        )
        self._pages = {0}
        self._selected = 0

    def as_fragment(self):
        return DisplayFragment(
            data=self.buffer, roi=self.roi, page=self._page, z=self._z,
            _ttl=self._ttl, _time=rospy.Time.now().to_sec()
        )

    def update(self, pages: Iterable[int], page: int):
        self._pages = pages
        self._selected = page if page in pages else 0

    def _render(self):
        # clear buffer
        self._buffer.fill(0)
        # render dots
        ch, cw = self.shape
        _, sw = self.SELECTED_PAGE_ICON.shape
        num_pages = len(self._pages)
        expected_w = num_pages * sw + (num_pages - 1) * self.SPACING_PX
        offset = int(np.floor((cw - expected_w) * 0.5))
        for page in sorted(self._pages):
            icon = self.SELECTED_PAGE_ICON if page == self._selected else self.UNSELECTED_PAGE_ICON
            self._buffer[:, offset:offset + sw] = icon
            offset += sw + self.SPACING_PX


if __name__ == '__main__':
    node = DisplayNode()
    rospy.spin()
