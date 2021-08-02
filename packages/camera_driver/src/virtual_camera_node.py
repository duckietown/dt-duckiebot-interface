#!/usr/bin/env python3

import os

import rospy

from camera_driver import AbsCameraNode
from sensor_msgs.msg import CompressedImage

from dt_duckiematrix_protocols.world.CameraFrame import CameraFrame
from dt_duckiematrix_utils.socket import DuckieMatrixSocket
from dt_robot_utils import get_robot_name


class VirtualCamera(AbsCameraNode):
    """
    Handles the imagery on a Virtual robot.
    """

    def __init__(self):
        # Initialize the DTROS parent class
        super(VirtualCamera, self).__init__()
        # prepare zmq pipeline
        self._device: DuckieMatrixSocket = DuckieMatrixSocket.create()
        if self._device is None or not self._device.connected:
            self.log("[VirtualCamera]: No virtual camera connection established.")
        else:
            self.log("[VirtualCamera]: Initialized.")

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
        if self._device is None or not self._device.connected:
            self.logerr('Device was found disconnected')
            return
        # run socket thread and join
        self._device.start()
        self._device.join()
        # ---
        self.loginfo('Camera worker stopped.')

    def setup(self):
        # setup camera
        camera_0_topic = os.path.join(get_robot_name(), "camera_0")
        self._device.subscribe(camera_0_topic, CameraFrame, self.process_frame)

    def release(self, force: bool = False):
        if self._device is not None:
            self.loginfo('Releasing camera...')
            self._device.release()
            self.loginfo('Camera released.')
        self._device = None


if __name__ == '__main__':
    # initialize the node
    camera_node = VirtualCamera()
    camera_node.start()
    # keep the node alive
    rospy.spin()
