#!/usr/bin/env python3
import os.path

import rospy
import uuid
import tf
import yaml

from geometry_msgs.msg import TransformStamped, Transform, Quaternion
from tf2_ros import TransformBroadcaster
from math import pi

from std_msgs.msg import Header
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from wheel_encoder import WheelEncoderDriver, WheelDirection
from duckietown.dtros import DTROS, TopicType, NodeType, DTParam, ParamType

from abc import ABC, abstractmethod
from typing import Optional, Dict, List, Union
from std_srvs.srv import Trigger, TriggerResponse
from enum import Enum
import json


class HWTestJsonParamType(Enum):
    STRING = 'string'
    BASE64 = 'base64'
    HTML = 'html'
    OBJECT = 'object'
    STREAM = 'stream'


class EnumJSONEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, Enum):
            return obj.value
        return super().default(obj)


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
        return self.html_util_ul(["Click on the <strong>Run the test</strong> button below."])

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
        return [
            self.format_obj(
                key="Preparation",
                value_type=HWTestJsonParamType.HTML,
                value=self.test_desc_preparation(),
            ),
            self.format_obj("Expected Outcomes", HWTestJsonParamType.HTML, self.test_desc_expectation()),
            self.format_obj("How to run", HWTestJsonParamType.HTML, self.test_desc_running()),
            self.format_obj("Logs Gathering (in case of errors)", HWTestJsonParamType.HTML, self.test_desc_log_gather()),
        ]

    @staticmethod
    def format_obj(key: str, value_type: "HWTestJsonParamType", value: str):
        return {
            "key": key,
            "type": value_type,
            "value": value,
        }

    @staticmethod
    def format_response_stream(
        success: bool,
        test_topic_name: str,
        test_topic_type: str,
        lst_blocks,
    ):
        ret_obj = {
            "type": HWTestJsonParamType.STREAM,
            "parameters": []
        }
        for block in lst_blocks:
            ret_obj["parameters"].append(block)

        ret_obj["parameters"].append(HWTest.format_obj(
            key="test_topic_name",
            value_type="str",
            value=test_topic_name,
        ))

        ret_obj["parameters"].append(HWTest.format_obj(
            key="test_topic_type",
            value_type="str",
            value=test_topic_type,
        ))

        return TriggerResponse(
            success=success,
            message=json.dumps(ret_obj, cls=EnumJSONEncoder),
        )

    @staticmethod
    def format_response_object(success: bool, lst_blocks):
        ret_obj = {
            "type": HWTestJsonParamType.OBJECT,
            "parameters": []
        }
        for block in lst_blocks:
            ret_obj["parameters"].append(block)

        return TriggerResponse(
            success=success,
            message=json.dumps(ret_obj, cls=EnumJSONEncoder),
        )

    @staticmethod
    def html_util_ul(lst_items: List[str]) -> str:
        ret = ["<ul>"]
        for item in lst_items:
            ret.append("<li>" + item + "</li>")
        ret.append("</ul>")
        return "".join(ret)

    def srv_cb_tst_desc(self, _):
        return self.format_response_object(
            success=True,
            lst_blocks=self.test_desc(),
        )


class HWTestWheelEncoder(HWTest):
    def __init__(self, wheel_side: str) -> None:
        super().__init__()
        self._desc_tst_srv = rospy.Service('~test/desc', Trigger, self.srv_cb_tst_desc)
        self._tst_srv = rospy.Service('~test/run', Trigger, self._tst)
        # test settings
        # attr
        self._name = wheel_side

    def _tst(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")

        instructions = self.html_util_ul([
            f"Now spin the <strong>{self._name}</strong> wheel by hand.",
            "Once your decide the test has passed/failed, you may mark the decision, and close this modal.",
        ])

        # Return the service response
        return self.format_response_stream(
            success=True,  # does not matter here
            test_topic_name=f"{self._name}_wheel_encoder_node/tick",
            test_topic_type="duckietown_msgs/WheelEncoderStamped",
            lst_blocks=[
                self.format_obj(
                    key="Instructions",
                    value_type=HWTestJsonParamType.HTML,
                    value=instructions,
                ),
            ],
        )

    def test_id(self) -> str:
        return f"Wheel Encoder ({self._name})"

    def test_desc_preparation(self) -> str:
        return self.html_util_ul([
            "Put your Duckiebot upside-down, where you can reach and turn the wheels by hand."
        ])

    def test_desc_expectation(self) -> str:
        return self.html_util_ul([
            "Once your start the test, a <strong>Tick value</strong> field will appear below.",
            "When you turn the wheel, if the tick value below changes according to your movements and rate, the test is passed."
        ])

    def test_desc_log_gather(self) -> str:
        return self.html_util_ul([
            "On your laptop, run the following command to save the logs.",
            "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
            "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
        ])

    def test_params(self) -> str:
        return f"[{self.test_id()}] N/A"


class WheelEncoderNode(DTROS):
    """Node handling a single wheel encoder.

    This node is responsible for reading data off of a single wheel encoders.
    Robots with N wheels will need to spin N instances of this node.
    This node is compatible with any rotary encoder that signals ticks as rising edges
    on a digital GPIO pin.

    For now we use the `wheel_cmd_executed` to determine if we are moving forwards or backwards.
    As a result, if you manually push the robot, you will get potentially incorrect output
    (we default to always forward in this case).

    Subscribers:
       ~wheels_cmd_executed (:obj:`WheemsCmdStamped`): The actual commands executed
    Publishers:
       ~data (:obj:`WheelEncoderStamped`): Publishes the cumulative number of ticks
                                            generated by the encoder.

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(WheelEncoderNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)
        # get parameters
        self._veh = rospy.get_param("~veh")
        self._name = rospy.get_param("~name")
        self._gpio_pin = rospy.get_param("~gpio")
        self._resolution = rospy.get_param("~resolution")
        self._configuration = rospy.get_param("~configuration")
        self._publish_frequency = DTParam(
            "~publish_frequency", param_type=ParamType.FLOAT, min_value=1.0, max_value=100.0
        )

        # register a callback for when publish_frequency changes
        self._publish_frequency.register_update_callback(self._frequency_change_cb)

        # try using custom calibration file
        calib_file = os.path.join(
            "/data/config/calibrations/encoder",
            f"{self._configuration}/{self._veh}.yaml",
        )
        try:
            with open(calib_file, "r") as f:
                calib_data = yaml.safe_load(f)
            custom_resolution = int(calib_data["resolution"])
            rospy.set_param("~resolution", custom_resolution)
            self._resolution = custom_resolution
            self.loginfo(
                (
                    f"With calibration file - {calib_file}, "
                    f"use custom encoder resolution: {self._resolution}"
                )
            )
        except FileNotFoundError:
            self.logwarn(
                (f"No custom encoder calibration found at: {calib_file}. " "Using default parameters.")
            )
        except KeyError:
            self.logwarn(
                (
                    "No valid field 'resolution' found in "
                    f"encoder calibration file at: {calib_file}. "
                    "Using default parameters."
                )
            )
        except ValueError:
            self.logwarn(
                (
                    "No valid integer 'resolution' value found in "
                    f"encoder calibration file at: {calib_file}. "
                    "Using default parameters."
                )
            )
        # throw exceptions for other situations

        # tick storage
        self._tick = 0
        # publisher for wheel encoder ticks
        self._tick_pub = rospy.Publisher(
            "~tick", WheelEncoderStamped, queue_size=1, dt_topic_type=TopicType.DRIVER
        )
        # subscriber for the wheel command executed
        self.sub_wheels = rospy.Subscriber(
            "~wheels_cmd_executed", WheelsCmdStamped, self._wheels_cmd_executed_cb, queue_size=1
        )
        # tf broadcaster for wheel frame
        self._tf_broadcaster = TransformBroadcaster()
        # setup a timer
        self._timer = rospy.Timer(rospy.Duration(1.0 / self._publish_frequency.value), self._cb_publish)
        # setup the driver
        self._driver = WheelEncoderDriver(self._gpio_pin, self._encoder_tick_cb)
        # hardware test
        self._hw_test = HWTestWheelEncoder(wheel_side=self._name)

    def _wheels_cmd_executed_cb(self, msg):
        if self._configuration == "left":
            if msg.vel_left >= 0:
                self._driver.set_direction(WheelDirection.FORWARD)
            else:
                self._driver.set_direction(WheelDirection.REVERSE)
        elif self._configuration == "right":
            if msg.vel_right >= 0:
                self._driver.set_direction(WheelDirection.FORWARD)
            else:
                self._driver.set_direction(WheelDirection.REVERSE)

    def _encoder_tick_cb(self, tick_no):
        """
        Callback that receives new ticks from the encoder.

            Args:
                tick_no (int): cumulative total number of ticks
        """
        self._tick = tick_no

    def _frequency_change_cb(self):
        """
        Callback triggered when the publish frequency changes.
        """
        self._timer.shutdown()
        frequency = self._publish_frequency.value
        self._timer = rospy.Timer(rospy.Duration(1.0 / frequency), self._cb_publish)
        self.loginfo(f"Publish frequency now set to {frequency}Hz")

    def _cb_publish(self, _):
        # Create header with timestamp
        header = Header()
        header.frame_id = f"{self._veh}/{self._name}_wheel_axis"
        header.stamp = rospy.Time.now()
        # publish WheelEncoderStamped message
        self._tick_pub.publish(
            WheelEncoderStamped(
                header=header,
                data=self._tick,
                resolution=self._resolution,
                type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL,
            )
        )
        # publish TF
        angle = (float(self._tick) / float(self._resolution)) * 2 * pi
        quat = tf.transformations.quaternion_from_euler(0, angle, 0)
        self._tf_broadcaster.sendTransform(
            TransformStamped(
                header=header,
                child_frame_id=f"{self._veh}/{self._name}_wheel",
                transform=Transform(rotation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])),
            )
        )


if __name__ == "__main__":
    # Initialize the node with rospy
    rand = str(uuid.uuid4())[:8]
    node = WheelEncoderNode("wheel_encoder_%s" % (rand,))
    # Keep it spinning to keep the node alive
    rospy.spin()
