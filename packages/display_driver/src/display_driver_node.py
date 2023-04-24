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
    PAGE_TEST_OLED_DISPLAY,
    MonoImageFragmentRenderer,
    AbsDisplayFragmentRenderer,
    DisplayROI,
    DisplayFragment,
)
from display_renderer.text import monospace_screen

from duckietown.utils.image.pil import np_to_pil, pil_to_np
from duckietown.utils.image.ros import imgmsg_to_mono8

from abc import ABC, abstractmethod
from typing import Optional, Dict
from std_srvs.srv import Trigger, TriggerResponse


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
        return "Run the test"

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
        # TODO: use JSON and keys to separate sections
        return "\n\n".join([
            self.test_desc_preparation(),
            self.test_desc_expectation(),
            self.test_desc_running(),
            self.test_desc_log_gather(),
        ])

    @abstractmethod
    def run_test(self) -> Optional[bool]:  # TODO: decide whether auto grade or not
        """return True or False if the result could be determined within the test"""
        pass


class HWTestOledDisplay(HWTest):
    def __init__(self, driver_node: "DisplayNode") -> None:
        super().__init__()
        self._driver_node = driver_node
        # test settings
        self.dura_secs = 5
        self.disp_text = "OLED Display Test"

    def test_id(self) -> str:
        return f"OLED Display"

    def test_desc_preparation(self) -> str:
        return (
            "Put your Duckiebot in normal orientation.\n"
            "And make sure you can see the top OLED display."
        )

    def test_desc_expectation(self) -> str:
        return (
            f"The top display should show: {self.disp_text}.\n"
            f"In about {self.dura_secs} seconds, the homepage should be shown."
        )
    
    def test_desc_log_gather(self) -> str:
        return (
            "On your laptop, run the following command to save the logs.\n"
            "Replace the `[path/to/save]' to the directory path where you would like to save the logs.\n"
            "`docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt'"
        )

    def test_params(self) -> str:
        return f"[{self.test_id()}] dura_secs = {self.dura_secs}, disp_text = '{self.disp_text}'"

    def run_test(self) -> Optional[bool]:
        start_ts = rospy.Time.now()
        end_ts = start_ts + rospy.Duration(self.dura_secs)
        # show test display
        self._driver_node.show_tst_page(OLEDDisplayTestRenderer(
            disp_text=self.disp_text,
            dura_secs=self.dura_secs,
        ).as_msg())
        # run until specified time reached
        while rospy.Time.now() < end_ts:
            rospy.sleep(1.0)
        # go back to homepage
        self._driver_node.hide_tst_page()


class OLEDDisplayTestRenderer(MonoImageFragmentRenderer):
    def __init__(self, disp_text: str = "Testing the OLED Display", dura_secs: int = 3):
        super(OLEDDisplayTestRenderer, self).__init__(
            name=f"__oled_display_test__",
            page=PAGE_TEST_OLED_DISPLAY,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
            ttl=dura_secs,  # on shutdown, just need one fixed screen
        )

        contents = monospace_screen((self.roi.h, self.roi.w), disp_text, scale="hfill", align="center")
        self.data[:, :] = contents


class DisplayNode(DTROS):
    _REGIONS = {
        DisplayFragmentMsg.REGION_FULL: REGION_FULL,
        DisplayFragmentMsg.REGION_HEADER: REGION_HEADER,
        DisplayFragmentMsg.REGION_BODY: REGION_BODY,
        DisplayFragmentMsg.REGION_FOOTER: REGION_FOOTER,
    }
    _MAX_FREQUENCY_HZ = 5

    def __init__(self):
        super(DisplayNode, self).__init__(node_name="display_driver_node", node_type=NodeType.DRIVER)
        # get parameters
        self._veh = rospy.get_param("~veh")
        self._i2c_bus = rospy.get_param("~bus", 1)
        self._i2c_address = rospy.get_param("~address", 0x3C)
        self._frequency = rospy.get_param("~frequency", 1)
        # create a display handler
        serial = i2c(port=self._i2c_bus, address=self._i2c_address)
        self._display = ssd1306(serial)
        # page selector
        self._page = PAGE_HOME
        self._pages = {PAGE_HOME}
        # create buffers
        self._fragments = {k: dict() for k in self._REGIONS}
        self._buffer = np.zeros(
            (
                self._REGIONS[DisplayFragmentMsg.REGION_FULL].height,
                self._REGIONS[DisplayFragmentMsg.REGION_FULL].width,
            ),
            dtype=np.uint8,
        )
        self._fragments_lock = Semaphore(1)
        self._device_lock = Semaphore(1)
        # create subscribers
        self._fragments_sub = rospy.Subscriber(
            "~fragments",
            DisplayFragmentMsg,
            self._fragment_cb,
            queue_size=10,
            buf_size="4M",
            dt_topic_type=TopicType.DRIVER,
            dt_help="Data to display on the display",
        )
        self._button_sub = rospy.Subscriber(
            "~button", ButtonEventMsg, self._button_event_cb, queue_size=1, dt_help="Button event"
        )
        # create internal renderers
        self._pager_renderer = PagerFragmentRenderer()
        # create rendering loop
        self._timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self._frequency), self._render)
        self._reminder = DTReminder(frequency=self._MAX_FREQUENCY_HZ)

        # hwtest
        self._desc_tst_srv = rospy.Service('~tests/oled_display/desc', Trigger, self._tst_desc)
        self._tst_srv = rospy.Service('~tests/oled_display/run', Trigger, self._tst)
        self._tst_def = None
        self._tst_page_id = None

    def _tst_desc(self, _):
        # this part is not in __init__ to make sure all initialization is completed
        if self._tst_def is None:
            self._tst_def = HWTestOledDisplay(self)
        return TriggerResponse(
            success=True,
            message=self._tst_def.test_desc(),
        )

    def show_tst_page(self, tst_page_msg):
        self._fragment_cb(tst_page_msg)
        with self._fragments_lock:
            self._page = PAGE_TEST_OLED_DISPLAY

    def hide_tst_page(self):
        with self._fragments_lock:
            self._page = PAGE_HOME

    def _tst(self, _):
        if self._tst_def is None:
            self._tst_def = HWTestOledDisplay(self)
        logs = []
        success = True

        try:
            test = self._tst_def
            self.log(f"[{test.test_id()}] Started")
            self.log(test.test_params())
            logs.append(f"[{test.test_id()}] Started")
            logs.append(test.test_params())
            test.run_test()
            self.log(f"[{test.test_id()}] Finished")
            logs.append(f"[{test.test_id()}] Finished")
        except Exception as e:
            logs.append(f"Exception occured. Details: {e}")
            success = False

        return TriggerResponse(
            success=success,
            message="\n".join(logs),
        )

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
                data=img, roi=roi, page=msg.page, z=msg.z, _ttl=msg.ttl, _time=msg.header.stamp.to_sec()
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
                        self.logdebug(
                            f"Fragment `{fragment_id}` on page `{fragment.page}`, "
                            f"region `{region}` w/ TTL `{fragment.given_ttl}` "
                            f"expired, remove!"
                        )
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
                    fragment for fragment in fragments.values() if fragment.page in [ALL_PAGES, self._page]
                ]
                for region, fragments in self._fragments.items()
            }
        # add pager fragment
        self._pager_renderer.update(self._pages, self._page)
        if self._pager_renderer.page in [ALL_PAGES, self._page]:
            data[self._pager_renderer.region.id].append(self._pager_renderer.as_fragment())
        # sort fragments by z-index
        data = {region: sorted(fragments, key=lambda f: f.z) for region, fragments in data.items()}
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
                self._buffer[fy : fy + fh, fx : fx + fw] = fragment.data
        # convert buffer to 1-byte pixel
        buf = Image.fromarray(self._buffer, mode="L")
        # convert 1-byte pixel to 1-bit pixel
        buf = buf.convert(mode="1")
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
        buf = Image.fromarray(self._buffer, mode="1")
        self.loginfo("Clearing display...")
        with self._device_lock:
            try:
                self._display.display(buf)
            except BlockingIOError:
                pass


class PagerFragmentRenderer(AbsDisplayFragmentRenderer):

    SPACING_PX = 7
    UNSELECTED_PAGE_ICON = np.array(
        [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 255, 255, 255, 0],
        ]
    )
    SELECTED_PAGE_ICON = np.array(
        [[0, 0, 0, 0, 0], [255, 255, 255, 255, 255], [255, 255, 255, 255, 255], [255, 255, 255, 255, 255]]
    )

    def __init__(self):
        super(PagerFragmentRenderer, self).__init__(
            "pager",
            page=ALL_PAGES,
            region=REGION_FOOTER,
            roi=DisplayROI(0, 0, REGION_FOOTER.width, REGION_FOOTER.height),
        )
        self._pages = {0}
        self._selected = 0

    def as_fragment(self):
        return DisplayFragment(
            data=self.buffer,
            roi=self.roi,
            page=self._page,
            z=self._z,
            _ttl=self._ttl,
            _time=rospy.Time.now().to_sec(),
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
            self._buffer[:, offset : offset + sw] = icon
            offset += sw + self.SPACING_PX


if __name__ == "__main__":
    node = DisplayNode()
    rospy.spin()
