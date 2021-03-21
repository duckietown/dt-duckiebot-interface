#!/usr/bin/env python3

import time
import rospy

from duckietown_msgs.msg import (
    ButtonEvent as ButtonEventMsg,
    DisplayFragment,
)

from duckietown.dtros import DTROS, NodeType, TopicType

from button_driver import ButtonEvent, ButtonDriver

from dt_device_utils.device import shutdown_device

# for shutting down the front and back LEDs
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import String
from duckietown.dtros.utils import apply_namespace

# display renderer for shutdown confirmation
from display_renderer import (
    PAGE_SHUTDOWN,
    MonoImageFragmentRenderer,
    REGION_BODY,
    DisplayROI,
)
from display_renderer.text import monospace_screen


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
        self._button.led.on()
        # display to confirm shutdown
        self._renderer = BatteryShutdownConfirmationRenderer()
        self._display_pub = rospy.Publisher(
            "~fragments",
            DisplayFragment,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION,
            dt_help="Fragments to display on the display"
        )
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
            # publish a display showing shutdown confirmation
            self._display_pub.publish(self._renderer.as_msg())
            time.sleep(1)
            self._publish(ButtonEventMsg.EVENT_HELD_3SEC)
            self._react(ButtonEventMsg.EVENT_HELD_3SEC)
            return
        # - held for 10 secs
        if self._TIME_HOLD_10S < duration:
            self._publish(ButtonEventMsg.EVENT_HELD_10SEC)
            self._react(ButtonEventMsg.EVENT_HELD_10SEC)
            return

    def _publish(self, event: int):
        self._pub.publish(ButtonEventMsg(event))

    def _react(self, event: int):
        if event in [ButtonEventMsg.EVENT_HELD_3SEC, ButtonEventMsg.EVENT_HELD_10SEC]:
            # blink top power button as a confirmation, too
            self._button.led.confirm_shutdown()

            # turn off front and back LEDs
            try:
                srv = rospy.ServiceProxy(
                    apply_namespace("led_emitter_node/set_pattern", ns_level=1),
                    ChangePattern,
                )
                msg = String()
                msg.data = "LIGHT_OFF"
                resp = srv(msg)
                self.loginfo(str(resp))
            except rospy.ServiceException as e:
                # not a big deal if failed this 
                self.logerr("LED shutdown service call failed {}".format(e))

            time.sleep(1)
            # init shutdown sequence
            res = shutdown_device()
            if not res:
                self.logerr("Could not initialize the shutdown sequence")

    def on_shutdown(self):
        if hasattr(self, '_button'):
            self._button.shutdown()


class BatteryShutdownConfirmationRenderer(MonoImageFragmentRenderer):

    def __init__(self):
        super(BatteryShutdownConfirmationRenderer, self).__init__(
            name=f'__battery_shutdown_confirmation__',
            page=PAGE_SHUTDOWN,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
            ttl=-1,  # on shutdown, just need one fixed screen
        )

        contents = monospace_screen(
            (self.roi.h, self.roi.w),
            "Shutting down", scale='hfill', align='center'
        )
        self.data[:,:] = contents 


if __name__ == '__main__':
    node = ButtonDriverNode()
    rospy.spin()
