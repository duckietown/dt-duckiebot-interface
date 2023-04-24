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


class HWTestButton(HWTest):
    def __init__(self, driver: ButtonDriver) -> None:
        super().__init__()
        self._driver = driver
        # test settings
        self.led_blink_secs = 5
        self.led_blink_hz = 1

    def test_id(self) -> str:
        return f"Top button"

    def test_desc_preparation(self) -> str:
        return "Put your Duckiebot in normal orientation, and make sure you can see the top button."

    def test_desc_expectation(self) -> str:
        return (
            f"The top button's LED should start blinking at {self.led_blink_hz} HZ.\n"
            f"In about {self.led_blink_secs} seconds, it should stop blinking."
        )
    
    def test_desc_log_gather(self) -> str:
        return (
            "On your laptop, run the following command to save the logs.\n"
            "Replace the `[path/to/save]' to the directory path where you would like to save the logs.\n"
            "`docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt'"
        )

    def test_params(self) -> str:
        return f"[{self.test_id()}] led_blink_secs = {self.led_blink_secs}, led_blink_hz = {self.led_blink_hz}"

    def run_test(self) -> Optional[bool]:
        self._driver.led.blink_led(secs_to_blink=self.led_blink_secs, blink_freq_hz=self.led_blink_hz)


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
        # display to confirm shutdown
        self._renderer = BatteryShutdownConfirmationRenderer()
        self._display_pub = rospy.Publisher(
            "~fragments",
            DisplayFragment,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION,
            dt_help="Fragments to display on the display",
        )
        # create event holder
        self._ongoing_event = None

        # hwtest
        self._desc_tst_srv = rospy.Service('~tests/top_button_led/desc', Trigger, self._tst_desc)
        self._tst_srv = rospy.Service('~tests/top_button_led/run', Trigger, self._tst)
        self._tst_def = None

    def _tst_desc(self, _):
        # this part is not in __init__ to make sure all initialization is completed
        if self._tst_def is None:
            self._tst_def = HWTestButton(self._button)
        return TriggerResponse(
            success=True,
            message=self._tst_def.test_desc(),
        )

    def _tst(self, _):
        if self._tst_def is None:
            self._tst_def = HWTestButton(self._button)
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
