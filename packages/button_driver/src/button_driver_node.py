#!/usr/bin/env python3

import time
import rospy

from duckietown_msgs.msg import (
    ButtonEvent as ButtonEventMsg,
    DisplayFragment,
)

from duckietown.dtros import DTROS, NodeType, TopicType

from button_driver import ButtonEvent, ButtonDriver
from hardware_test_button import HardwareTestButton

from dt_device_utils.device import shutdown_device

# for shutting down the front and back LEDs
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse
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
        super(ButtonDriverNode, self).__init__(node_name="button_driver_node", node_type=NodeType.DRIVER)
        # get parameters
        self._led_gpio_pin = rospy.get_param("~led_gpio_pin")
        self._signal_gpio_pin = rospy.get_param("~signal_gpio_pin")
        # create publishers
        self._pub = rospy.Publisher(
            "~event", ButtonEventMsg, queue_size=1, dt_topic_type=TopicType.DRIVER, dt_help="Button event"
        )
        # create button driver
        self._button = ButtonDriver(self._led_gpio_pin, self._signal_gpio_pin, self._event_cb)
        self._button.led.on()

        # shutdown confirmation service (for external methods of shutting down, e.g. dts/Dashboard):
        #   turn off LED, blink power LED, show Shutdown page on Display
        self._srv_shutdown_behavior = rospy.Service(
            "~shutdown_behavior", Trigger, self._srv_cb_shutdown_behavior,
        )
        # create event holder
        self._ongoing_event = None

        # user hardware test
        self._hardware_test = HardwareTestButton(driver=self._button)

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
            self._show_shutdown_behavior()
            # init shutdown sequence
            res = shutdown_device()
            if not res:
                self.logerr("Could not initialize the shutdown sequence")
    
    def _show_shutdown_behavior(self):
        # the page confirming shutdown
        _renderer = BatteryShutdownConfirmationRenderer()
        # publish a display showing shutdown confirmation
        _display_pub = rospy.Publisher(
            "~fragments",
            DisplayFragment,
            latch=True,
            queue_size=1,
        )
        _display_pub.publish(_renderer.as_msg())
        n_times_to_try = 3
        # try several times to go to page
        for _ in range(n_times_to_try):
            # emulate button event to switch to shutdown page
            self._publish(ButtonEventMsg.EVENT_HELD_3SEC)
            rospy.sleep(1.0 / n_times_to_try)

        # turn off LEDs
        _led_pub = rospy.Publisher(
            apply_namespace("led_emitter_node/led_pattern", ns_level=1),
            LEDPattern,
            latch=True,
            queue_size=1,
        )
        msg_led = LEDPattern()
        msg_rgba = ColorRGBA(r=0, g=0, b=0, a=0)
        msg_led.rgb_vals = [msg_rgba] * 5
        _led_pub.publish(msg_led)

        # blink top power button as a confirmation, too
        self._button.led.confirm_shutdown()

        rospy.sleep(1)

    def _srv_cb_shutdown_behavior(self, _):
        # for external methods of shutting down (e.g. from dts or the Dashboard)
        try:
            self._show_shutdown_behavior()
            return TriggerResponse(success=True, message="")
        except Exception as e:
            return TriggerResponse(
                success=False,
                message=f"Failed to show shutdown behaviors. Reason: {e}",
            )

    def on_shutdown(self):
        if hasattr(self, "_button"):
            self._button.shutdown()


class BatteryShutdownConfirmationRenderer(MonoImageFragmentRenderer):
    def __init__(self):
        super(BatteryShutdownConfirmationRenderer, self).__init__(
            name=f"__battery_shutdown_confirmation__",
            page=PAGE_SHUTDOWN,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
            ttl=-1,  # on shutdown, just need one fixed screen
        )

        contents = monospace_screen((self.roi.h, self.roi.w), "Shutting down", scale="hfill", align="center")
        self.data[:, :] = contents


if __name__ == "__main__":
    node = ButtonDriverNode()
    rospy.spin()
