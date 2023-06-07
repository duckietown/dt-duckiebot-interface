#!/usr/bin/env python3

from threading import Condition
from typing import Optional

import rospy
from camera_driver import AbsCameraNode
from sensor_msgs.msg import CompressedImage

from dt_duckiematrix_messages.CameraFrame import CameraFrame
from dt_duckiematrix_protocols import Matrix
from dt_duckiematrix_protocols.robot.features.sensors import Camera
from dt_duckiematrix_utils.ros import \
    on_duckiematrix_connection_request, \
    DuckiematrixLinkDescription
from dt_robot_utils import get_robot_configuration


class VirtualCamera(AbsCameraNode):
    """
    Handles the imagery on a Virtual robot.
    """

    def __init__(self):
        # Initialize the DTROS parent class
        super(VirtualCamera, self).__init__()
        self._matrix: Optional[Matrix] = None
        self._device: Optional[Camera] = None
        # register connection setup function
        self._connection_request: Optional[DuckiematrixLinkDescription] = None
        self._new_connection_request = Condition()
        on_duckiematrix_connection_request(self.on_connection_request)

    def on_connection_request(self, link: DuckiematrixLinkDescription):
        self.log(f"[VirtualCamera]: Received request to connect to Duckiematrix '{link.matrix}'.")
        # store new connection request
        self._connection_request = link
        # notify node of the new connection
        with self._new_connection_request:
            self._new_connection_request.notify()

    def setup(self):
        if self._connection_request is None:
            return
        # ---
        link = self._connection_request
        # prepare zmq pipeline
        self.log(f"[VirtualCamera]: Connecting to Matrix Engine at '{link.uri}'.")
        self._matrix: Matrix = Matrix(link.uri)
        self._device: Camera = self._matrix.robots.CameraEnabledRobot(link.entity).camera
        # subscribe to camera topic
        self._device.start()
        self.log(f"[VirtualCamera]: Sensor started.")

    def run(self):
        """ Image capture procedure.
        """
        if self._device is None:
            # wait for connection request
            self.loginfo("Waiting for connection request...")
            with self._new_connection_request:
                self._new_connection_request.wait()
                self.setup()
                self.loginfo("[VirtualCamera]: Initialized.")
        # do this forever
        while (not self.is_stopped) and (not self.is_shutdown):
            msg = self._device.capture(block=True)
            self.process_frame(msg)

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

    def release(self, force: bool = False):
        self.loginfo('Releasing camera...')
        # unsubscribe from camera topic
        self._device.detach(self.process_frame)
        self._device.release()
        self.loginfo('Camera released.')
        self._device = None


if __name__ == '__main__':
    # initialize the node
    camera_node = VirtualCamera()
    camera_node.start()
    # keep the node alive
    rospy.spin()
