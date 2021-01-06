#!/usr/bin/env python3

import os
from typing import Union

import cv2
import rospy
import requests
from cv_bridge import CvBridge
from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg

from display_renderer import \
    REGION_HEADER, \
    REGION_BODY, \
    DisplayROI, \
    TextFragmentRenderer, \
    AbsDisplayFragmentRenderer, \
    Z_SYSTEM
from display_renderer.text import monospace_screen
from duckietown.dtros import DTROS, NodeType, TopicType


class HealthDisplayRendererNode(DTROS):

    def __init__(self):
        super(HealthDisplayRendererNode, self).__init__(
            node_name='health_renderer_node',
            node_type=NodeType.VISUALIZATION
        )
        # get parameters
        self._veh = rospy.get_param('~veh')
        self._assets_dir = rospy.get_param('~assets_dir')
        self._frequency = rospy.get_param('~frequency')
        # create a cv bridge instance
        self._bridge = CvBridge()
        # create publisher
        self._pub = rospy.Publisher(
            "~fragments",
            DisplayFragmentMsg,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION,
            dt_help="Fragments to display on the display"
        )
        # create renderers
        self._battery_indicator = BatteryIndicatorFragmentRenderer(self._assets_dir)
        self._usage_renderer = UsageStatsFragmentRenderer()
        self._renderers = [
            self._battery_indicator,
            self._usage_renderer
        ]
        # create loop
        self._timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self._frequency), self._beat)

    def _beat(self, _):
        self._fetch()
        self._publish()

    def _fetch(self):
        health_api_url = f"http://{self._veh}.local/health/"
        try:
            health_data = requests.get(health_api_url).json()
        except:
            return
        self._usage_renderer.set(
            ctmp=health_data['temp'],
            pcpu=health_data['pcpu'],
            pmem=health_data['mem']['pmem'],
            pdsk=health_data['disk']['pdisk'],
        )
        self._battery_indicator.update(
            present=isinstance(health_data['battery']['cycle_count'], int),
            charging=health_data['battery']['input_voltage'] > 3.0,
            percentage=health_data['battery']['percentage']
        )

    def _publish(self):
        for renderer in self._renderers:
            self._pub.publish(renderer.as_msg())


class BatteryIndicatorFragmentRenderer(AbsDisplayFragmentRenderer):

    def __init__(self, assets_dir: str):
        super(BatteryIndicatorFragmentRenderer, self).__init__(
            '__battery_indicator__',
            screen=0,
            region=REGION_HEADER,
            roi=DisplayROI(90, 0, 38, 16),
            z=Z_SYSTEM
        )
        self._assets_dir = assets_dir
        self._percentage = 0
        self._charging = False
        self._present = False
        # load assets
        _asset_path = lambda a: os.path.join(self._assets_dir, 'icons', f'{a}.png')
        self._assets = {
            asset: cv2.imread(_asset_path(asset), cv2.IMREAD_GRAYSCALE)
            for asset in [
                'battery_not_found',
                'battery_charging',
                'battery_0',
                'battery_1',
                'battery_2',
                'battery_3',
                'battery_4',
                'battery_5',
                'battery_6',
                'battery_7',
                'battery_8',
                'battery_9',
                'battery_10',
            ]
        }

    def update(self, present: bool = None, charging: bool = None, percentage: int = None):
        if present is not None:
            self._present = present
        if charging is not None:
            self._charging = charging
        if percentage is not None:
            self._percentage = percentage

    def _render(self):
        def _indicator(icon: str, text: str):
            batt_not_found_icon = self._assets[icon]
            ico_h, ico_w = batt_not_found_icon.shape
            ico_space = 2
            # draw icon
            self._buffer[0:ico_h, 0:ico_w] = batt_not_found_icon
            # draw text
            text_h, text_w = 12, self._roi.w - ico_w - ico_space
            text_buf = monospace_screen((text_h, text_w), text, scale='vfill')
            self._buffer[3:3 + text_h, ico_w + ico_space:] = text_buf

        # battery not found
        if not self._present:
            _indicator('battery_not_found', 'NoBT')
            return
        # battery charging
        if self._charging:
            _icon = 'battery_charging'
        else:
            dec = '%d' % (self._percentage / 10)
            _icon = f'battery_{dec}'
        # battery discharging
        _indicator(_icon, '%d%%' % self._percentage)


class UsageStatsFragmentRenderer(TextFragmentRenderer):
    BAR_LEN = 14
    CANVAS = """\
TEMP |{ctmp_bar}| {ctmp}
CPU  |{pcpu_bar}| {pcpu}
RAM  |{pmem_bar}| {pmem}
DISK |{pdsk_bar}| {pdsk}
"""

    def __init__(self):
        super(UsageStatsFragmentRenderer, self).__init__(
            '__usage_stats__',
            screen=0,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
            scale='vfill'
        )
        self._min_ctmp = 30
        self._max_ctmp = 100

    def set(self, ctmp: Union[str, int], pcpu: Union[str, int], pmem: Union[str, int], pdsk: Union[str, int]):
        ptmp = int(100 * (max(0, ctmp - self._min_ctmp) / (self._max_ctmp - self._min_ctmp))) \
            if isinstance(ctmp, int) else 0
        text = self.CANVAS.format(**{
            'ctmp': self._fmt(ctmp, 'C'),
            'pcpu': self._fmt(pcpu, '%'),
            'pmem': self._fmt(pmem, '%'),
            'pdsk': self._fmt(pdsk, '%'),
            'ctmp_bar': self._bar(ptmp, scale=1),
            'pcpu_bar': self._bar(pcpu),
            'pmem_bar': self._bar(pmem),
            'pdsk_bar': self._bar(pdsk)
        })
        self.update(text)

    @staticmethod
    def _fmt(value: Union[str, int], suffix: str):
        if isinstance(value, str):
            return f"ERR"
        return f"{int(value)}{suffix}"

    @classmethod
    def _bar(cls, value: Union[str, int], scale: int = 100):
        if isinstance(value, str):
            return f"ERR"
        value /= scale
        full = int(cls.BAR_LEN * value)
        return "|" * full + " " * (cls.BAR_LEN - full)


if __name__ == '__main__':
    node = HealthDisplayRendererNode()
    rospy.spin()