#!/usr/bin/env python3
import dataclasses
import time
from collections import defaultdict

import rospy

from duckietown_msgs.msg import ButtonEvent as ButtonEventMsg

from duckietown.dtros import DTROS, NodeType, TopicType

from button_driver import ButtonEvent, ButtonDriver


@dataclasses.dataclass
class FullButtonEvent:
    press_time: float
    release_time: float

    @property
    def duration(self):
        return self.release_time - self.press_time

    def __le__(self, other):
        return self.duration <= other

    def __ge__(self, other):
        return self.duration >= other

    def __lt__(self, other):
        return self.duration < other

    def __gt__(self, other):
        return self.duration > other

    def __sub__(self, other) -> float:
        if not isinstance(other, FullButtonEvent):
            raise ValueError('Expected object of type `FullButtonEvent`')
        return self.release_time - other.release_time


class ButtonDriverNode(DTROS):

    _TIME_DOUBLE_CLICK_S = 0.1
    _TIME_HOLD_3S = 3
    _TIME_HOLD_10S = 10

    def __init__(self):
        super(ButtonDriverNode, self).__init__(
            node_name='button_driver_node',
            node_type=NodeType.DRIVER
        )
        # get parameters
        self._led_gpio_pin = rospy.get_param('~led_gpio_pin')
        self._signal_gpio_pin = rospy.get_param('~signal_gpio_pin')
        # create publishers
        self._sub = rospy.Publisher(
            "~event",
            ButtonEventMsg,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="Button event"
        )
        # create button driver
        self._button = ButtonDriver(self._led_gpio_pin, self._signal_gpio_pin, self._event_cb)
        # create events history
        self._events = []
        self._ongoing_event = None

    def _time_between(self, event1: ButtonEvent, event2: ButtonEvent):
        if self._events[event2] == 0.0:
            return time.time()
        return self._events[event2] - self._events[event1]

    def _event_cb(self, event: ButtonEvent):
        # use the LED to mimic the event
        self._button.led.set(event.value)
        # create partial event
        if event == ButtonEvent.PRESS:
            # create new partial event
            self._ongoing_event = time.time()
            return
        # this is a RELEASE event
        # create new full event
        if self._ongoing_event is None:
            # we missed it, well, next time!
            return
        self._events.append(FullButtonEvent(
            press_time=self._ongoing_event, release_time=time.time()
        ))
        # clear ongoing event
        self._ongoing_event = None
        # need at least two events
        if len(self._events) <= 1:
            return
        # clear history
        self._events = self._events[-2:]
        # analyze history of events looking for known patterns
        evt1, evt2 = self._events[-1], self._events[-2]
        # - double click
        dclick_s = self._TIME_DOUBLE_CLICK_S
        print(evt1 - evt2)
        if evt1 - evt2 <= 2 * dclick_s and evt1 < dclick_s and evt2 < dclick_s:
            print('DOUBLE')
            return
        # - held for 3 secs
        if self._TIME_HOLD_3S < evt1 < 2 * self._TIME_HOLD_3S:
            print('HELD_3')
            return
        # - held for 10 secs
        if self._TIME_HOLD_10S < evt1 < 2 * self._TIME_HOLD_10S:
            print('HELD_10')
            return
        # - single click
        print('SINGLE')
        return

    def _publish(self, event: int):
        print(event)

    def on_shutdown(self):
        if hasattr(self, '_button'):
            self._button.shutdown()


if __name__ == '__main__':
    node = ButtonDriverNode()
    rospy.spin()
