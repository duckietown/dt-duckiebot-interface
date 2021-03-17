#!/usr/bin/env python3

import time
import rospy

from duckietown_msgs.msg import ButtonEvent as ButtonEventMsg

from duckietown.dtros import DTROS, NodeType, TopicType

from button_driver import ButtonEvent, ButtonDriver

from dt_device_utils.device import shutdown_device


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
        self._pub = rospy.Publisher(
            "~event",
            ButtonEventMsg,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="Button event"
        )
        # create button driver
        self._button = ButtonDriver(self._led_gpio_pin, self._signal_gpio_pin, self._event_cb)
        # create event holder
        self._ongoing_event = None

    def _event_cb(self, event: ButtonEvent):
        # create partial event
        if event == ButtonEvent.PRESS:
            # create new partial event
            self._ongoing_event = time.time()
            return
        # this is a RELEASE event
        if self._ongoing_event is None:
            # we missed it, well, next time!
            return
        # create new full event
        duration = time.time() - self._ongoing_event
        # clear ongoing event
        self._ongoing_event = None
        # analyze event
        # - single click
        if duration < 0.5:
            self._publish(ButtonEventMsg.EVENT_SINGLE_CLICK)
            self._react(ButtonEventMsg.EVENT_SINGLE_CLICK)
            return
        # - held for 3 secs
        if self._TIME_HOLD_3S < duration < 2 * self._TIME_HOLD_3S:
            self._publish(ButtonEventMsg.EVENT_HELD_3SEC)
            self._react(ButtonEventMsg.EVENT_HELD_3SEC)
            return
        # - held for 10 secs
        if self._TIME_HOLD_10S < duration < 2 * self._TIME_HOLD_10S:
            self._publish(ButtonEventMsg.EVENT_HELD_10SEC)
            self._react(ButtonEventMsg.EVENT_HELD_10SEC)
            return

    def _publish(self, event: int):
        self._pub.publish(ButtonEventMsg(event))

    def _react(self, event: int):
        if event == ButtonEventMsg.EVENT_HELD_3SEC:
            # TODO: publish a new screen for the display
            time.sleep(1)
            # init shutdown sequence
            res = shutdown_device()
            if not res:
                self.logerr("Could not initialize the shutdown sequence")

    def on_shutdown(self):
        if hasattr(self, '_button'):
            self._button.shutdown()


if __name__ == '__main__':
    node = ButtonDriverNode()
    rospy.spin()
