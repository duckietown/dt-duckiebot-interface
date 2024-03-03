#!/usr/bin/env python3

import time
from logging import Logger
from threading import Semaphore
from typing import Iterable

import numpy as np
from PIL import Image
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306

from display_renderer import AbsDisplayFragmentRenderer
from duckietown_messages.actuators.display_fragment import DisplayFragment as DisplayFragmentMsg
from duckietown_messages.utils.image.pil import np_to_pil, pil_to_np
from ..types.fragment import DisplayFragment
from ..types.page import ALL_PAGES, PAGE_HOME, PAGE_INIT
from ..types.regions import DisplayRegionID, DisplayRegion
from ..types.regions import REGION_FULL, REGION_HEADER, REGION_BODY, REGION_FOOTER
from ..types.roi import DisplayROI


class SSD1306Display:
    _REGIONS = {
        DisplayRegionID.FULL: REGION_FULL,
        DisplayRegionID.HEADER: REGION_HEADER,
        DisplayRegionID.BODY: REGION_BODY,
        DisplayRegionID.FOOTER: REGION_FOOTER,
    }

    def __init__(self, bus: int, address: int, frequency: float, logger: Logger):
        # get parameters
        self._i2c_bus: int = bus
        self._i2c_address: int = address
        self._frequency: float = frequency
        self._logger: Logger = logger
        # INIT page only shown until HOME is available
        self._inited: bool = False
        # create a display handler
        serial = i2c(port=self._i2c_bus, address=self._i2c_address)
        self._display = ssd1306(serial)
        # page selector
        self._page = PAGE_INIT
        self._pages = {PAGE_INIT}
        # create buffers
        self._fragments = {k: dict() for k in self._REGIONS}
        self._buffer = np.zeros(
            (
                self._REGIONS[DisplayRegionID.FULL].height,
                self._REGIONS[DisplayRegionID.FULL].width,
            ),
            dtype=np.uint8,
        )
        self._fragments_lock = Semaphore()
        self._device_lock = Semaphore()
        # create internal renderers
        self._pager_renderer = PagerFragmentRenderer()

    @property
    def page(self) -> int:
        return self._page

    @page.setter
    def page(self, page: int):
        with self._fragments_lock:
            self._page = page
        self.render()

    def next_page(self):
        with self._fragments_lock:
            pages = sorted([p for p in self._pages if p != PAGE_INIT])
            # move to the next page
            try:
                i = pages.index(self._page)
                i = pages[(i + 1) % len(pages)]
            except ValueError:
                i = 0
        # select page (and render)
        self.page = i

    def add_fragment(self, fragment: DisplayFragmentMsg):
        region_id: DisplayRegionID = DisplayRegionID(fragment.region)
        region: DisplayRegion = self._REGIONS[region_id]
        # convert image to greyscale
        img: np.ndarray = fragment.content.as_mono8()
        # parse ROI
        roi: DisplayROI = DisplayROI.from_message(fragment.location)
        if roi is None:
            # no ROI was specified
            # the fragment will be used as is if it fits the region, resized otherwise
            rh, rw = region.height, region.width
            fh, fw = img.shape
            # does it fit?
            if fw > rw or fh > rh:
                pimg = np_to_pil(img, mode="L")
                pimg = pimg.resize((rw, rh), resample=Image.NEAREST)
                img = pil_to_np(pimg)
                fh, fw = rh, rw
            roi = DisplayROI(x=0, y=0, w=fw, h=fh)
        else:
            # validate ROI
            fh, fw = img.shape
            # - find biggest offsets achievable
            fx, fy = min(region.width, roi.x), min(region.height, roi.y)
            # - find the biggest canvas size achievable
            cw, ch = min(region.width - fx, roi.w), min(region.height - fy, roi.h)
            # does it fit?
            if fw > cw or fh > ch:
                pimg = np_to_pil(img, mode="L")
                pimg = pimg.resize((cw, ch), resample=Image.NEAREST)
                img = pil_to_np(pimg)
                fh, fw = ch, cw
            # update ROI
            roi = DisplayROI(x=fx, y=fy, w=fw, h=fh)

        # threshold image at mid-range
        img = (img > 125).astype(np.uint8) * 255
        # list fragment for rendering
        with self._fragments_lock:
            self._fragments[region_id][fragment.name] = DisplayFragment(
                data=img, roi=roi, page=fragment.page, z=fragment.z, _ttl=fragment.ttl, _time=time.time()
            )

        # switch to HOME as soon as possible
        if not self._inited and fragment.page == PAGE_HOME:
            self._inited = True
            self.page = PAGE_HOME

        # force refresh if this fragment is on the current page
        if fragment.page == self._page:
            self.render()

    def render(self):
        with self._fragments_lock:
            # clean pages
            self._pages = {PAGE_HOME if self._inited else PAGE_INIT}
            # remove expired fragments and annotate how many pages we need
            for region, fragments in self._fragments.items():
                for fragment_id in set(fragments.keys()):
                    fragment = fragments[fragment_id]
                    if fragment.ttl() <= 0:
                        self._logger.debug(
                            f"Fragment `{fragment_id}` on page `{fragment.page}`, "
                            f"region `{region}` w/ TTL `{fragment.given_ttl}` "
                            f"expired, remove!"
                        )
                        del self._fragments[region][fragment_id]
                    if fragment.page != ALL_PAGES:
                        self._pages.add(fragment.page)
            # sanitize page selector
            if self._page not in self._pages:
                # go back to a known page
                self._page = PAGE_HOME if self._inited else PAGE_INIT
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
        buf = Image.fromarray(self._buffer, mode="L")
        # convert 1-byte pixel to 1-bit pixel
        buf = buf.convert(mode="1")
        # display buffer
        with self._device_lock:
            try:
                self._display.display(buf)
            except BlockingIOError:
                pass

    def release(self):
        self._logger.info("Clearing buffer...")
        # clear buffer
        self._buffer.fill(0)
        # render nothing
        buf = Image.fromarray(self._buffer, mode="1")
        self._logger.info("Clearing display...")
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
        [
            [0, 0, 0, 0, 0],
            [255, 255, 255, 255, 255],
            [255, 255, 255, 255, 255],
            [255, 255, 255, 255, 255]
        ]
    )

    def __init__(self):
        super(PagerFragmentRenderer, self).__init__(
            "pager",
            page=ALL_PAGES,
            region=REGION_FOOTER,
            roi=DisplayROI(0, 0, REGION_FOOTER.width, REGION_FOOTER.height),
        )
        self._pages = {}
        self._selected = None

    def as_fragment(self):
        return DisplayFragment(
            data=self.buffer,
            roi=self.roi,
            page=self._page,
            z=self._z,
            _ttl=self._ttl,
            _time=time.time(),
        )

    def update(self, pages: Iterable[int], page: int):
        self._pages = pages
        self._selected = page
        self.render()

    def render(self):
        self._clear_buffer()
        # sort pages; exclude the INIT page
        pages = [p for p in sorted(self._pages) if p != PAGE_INIT]
        # render dots
        ch, cw = self.shape
        _, sw = self.SELECTED_PAGE_ICON.shape
        num_pages = len(pages)
        expected_w = num_pages * sw + (num_pages - 1) * self.SPACING_PX
        offset = int(np.floor((cw - expected_w) * 0.5))
        for page in pages:
            icon = self.SELECTED_PAGE_ICON if page == self._selected else self.UNSELECTED_PAGE_ICON
            self._buffer[:, offset:offset + sw] = icon
            offset += sw + self.SPACING_PX
