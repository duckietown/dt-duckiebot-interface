#!/usr/bin/env python3

import os
from threading import Condition
from typing import Optional

import rospy

from camera_driver import AbsCameraNode
from sensor_msgs.msg import CompressedImage

from dt_duckiematrix_protocols.CameraFrame import CameraFrame
from dt_duckiematrix_utils.socket import DuckieMatrixSocket
from dt_duckiematrix_utils.ros import on_duckiematrix_connection_request
from dt_robot_utils import get_robot_name


class VirtualCamera(AbsCameraNode):
    """
    Handles the imagery on a Virtual robot.
    """

    def __init__(self):
        # Initialize the DTROS parent class
        super(VirtualCamera, self).__init__()
        self._device: Optional[DuckieMatrixSocket] = None
        self._camera_0_topic = os.path.join(get_robot_name(), "camera_0")
        # register connection setup function
        self._new_connection_request = Condition()
        on_duckiematrix_connection_request(self.on_connection_request)

    def on_connection_request(self, name: str, data_in_uri: str, data_out_uri: str):
        self.log(f"[VirtualCamera]: Received request to connect to Duckiematrix '{name}'.")
        # try to clear the current pipeline
        try:
            self.release()
        except BaseException:
            pass
        # prepare zmq pipeline
        self._device = DuckieMatrixSocket(data_in_uri, data_out_uri)
        if self._device is None or not self._device.connected:
            self.log("[VirtualCamera]: No virtual camera connection established.")
        # notify node of the new connection
        with self._new_connection_request:
            self._new_connection_request.notify()

    def process_frame(self, msg: CameraFrame):
        if msg.frame is None:
            return
        if msg.format != "jpeg":
            self.logerr(f"Format '{msg.format}' not supported.")
        # image is already JPEG encoded
        image_msg = CompressedImage()
        image_msg.header.stamp = rospy.Time.now()
        image_msg.format = "jpeg"
        image_msg.data = msg.frame
        # publish the compressed image
        self.publish(image_msg)

    def run(self):
        """ Image capture procedure.
        """
        if self._device is None:
            # wait for connection request
            self.loginfo('Waiting for connection request')
            with self._new_connection_request:
                self._new_connection_request.wait()
        # do this forever
        while True:
            # new request received
            if self._device is None or not self._device.connected:
                self.logerr('Device was found disconnected')
                return
            # start device
            self.setup()
            self._device.start()
            self.log("[VirtualCamera]: Initialized.")
            with self._new_connection_request:
                self._new_connection_request.wait()
            # ---
            self.loginfo('Camera worker stopped.')

    def setup(self):
        if self._device is None:
            return
        # subscribe to camera topic
        self._device.subscribe(self._camera_0_topic, CameraFrame, self.process_frame)

    def release(self, force: bool = False):
        if self._device is not None:
            self.loginfo('Releasing camera...')
            # unsubscribe from camera topic
            self._device.unsubscribe(self._camera_0_topic, self.process_frame)
            self._device.release()
            self.loginfo('Camera released.')
        self._device = None


if __name__ == '__main__':
    # initialize the node
    camera_node = VirtualCamera()
    camera_node.start()
    # keep the node alive
    rospy.spin()
