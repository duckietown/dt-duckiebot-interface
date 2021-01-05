#!/usr/bin/env python3

import abc
import rospy
import numpy as np

from cv_bridge import CvBridge
from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Header

from display_renderer import DisplayROI, DisplayRegion, monospace_screen
from typing import Union, Iterable


class AbsDisplayFragmentRenderer(abc.ABC):

    def __init__(self, name: str, screen: int, region: DisplayRegion, roi: DisplayROI,
                 ttl: int = 10, z: int = 0):
        self._name = name
        self._screen = screen
        self._region = region
        self._roi = roi
        self._ttl = ttl
        self._z = z
        self._buffer = np.zeros((self._roi.h, self._roi.w), dtype=np.uint8)
        self._bridge = CvBridge()

    @property
    def name(self) -> str:
        return self._name

    @property
    def shape(self) -> tuple:
        return self._roi.h, self._roi.w

    @property
    def roi(self) -> DisplayROI:
        return self._roi

    @property
    def buffer(self) -> np.ndarray:
        self._render()
        return self._buffer

    def as_msg(self):
        self._render()
        return DisplayFragmentMsg(
            header=Header(stamp=rospy.Time.now()),
            id=self._name,
            region=self._region.id,
            screen=self._screen,
            data=self._bridge.cv2_to_imgmsg(self._buffer, encoding="mono8"),
            location=RegionOfInterest(
                x_offset=self._roi.x, y_offset=self._roi.y, width=self._roi.w, height=self._roi.h
            ),
            ttl=self._ttl,
            z=self._z
        )

    def _clear_buffer(self):
        self._buffer.fill(0)

    @abc.abstractmethod
    def _render(self):
        pass


class TextFragmentRenderer(AbsDisplayFragmentRenderer):

    def __init__(self, name: str, screen: int, region: DisplayRegion, roi: DisplayROI,
                 *args, **kwargs):
        super(TextFragmentRenderer, self).__init__(name, screen, region, roi, *args, **kwargs)
        self._text = ""

    def update(self, text: Union[Iterable[str], str]):
        self._text = text

    def _render(self):
        self._buffer = monospace_screen((self._region.height, self._region.width), self._text)


class NumpyArrayFragmentRenderer(AbsDisplayFragmentRenderer):

    @property
    def data(self):
        return self._buffer

    def _render(self):
        pass
