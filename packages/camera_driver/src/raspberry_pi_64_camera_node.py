#!/usr/bin/env python3

import cv2
import rospy
import atexit
import subprocess
import numpy as np
from typing import cast

from camera_driver import AbsCameraNode
from sensor_msgs.msg import CompressedImage


class RaspberryPi64Camera(AbsCameraNode):
    """
    Handles the imagery on a Raspberry Pi 64.
    """

    VIDEO_DEVICE = "/dev/video0"

    def __init__(self):
        # Initialize the DTROS parent class
        super(RaspberryPi64Camera, self).__init__()
        # prepare gstreamer pipeline
        self._device = None
        # ---
        self.log("[RaspberryPi64Camera]: Initialized.")

    def run(self):
        """ Image capture procedure.

            Captures a frame from the /dev/video0 image sink and publishes it.
        """
        if self._device is None or not self._device.isOpened():
            self.logerr('Device was found closed')
            return
        # get first frame
        retval, image = self._device.read() if self._device else (False, None)
        # keep reading
        while (not self.is_stopped) and (not self.is_shutdown) and retval:
            if image is not None:
                # image is already JPEG encoded
                image = cast(np.ndarray, image)
                image_msg = CompressedImage()
                image_msg.header.stamp = rospy.Time.now()
                image_msg.format = "jpeg"
                image_msg.data = image.tobytes()
                # publish the compressed image
                self.publish(image_msg)
            # grab next frame
            retval, image = self._device.read() if self._device else (False, None)
        self.loginfo('Camera worker stopped.')

    def setup(self):
        # setup camera
        cam_props = {
            'video_bitrate': 25000000,
        }
        for key in cam_props:
            subprocess.call(
                f'v4l2-ctl -d {RaspberryPi64Camera.VIDEO_DEVICE} -c {key}={str(cam_props[key])}',
                shell=True
            )
        # create VideoCapture object
        if self._device is None:
            self._device = cv2.VideoCapture()
        # open the device
        if not self._device.isOpened():
            try:
                self._device.open(RaspberryPi64Camera.VIDEO_DEVICE, cv2.CAP_V4L2)
                # make sure the device is open
                if not self._device.isOpened():
                    msg = "OpenCV cannot open camera"
                    self.logerr(msg)
                    raise RuntimeError(msg)
                # configure camera
                self._device.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self._device.set(cv2.CAP_PROP_FRAME_WIDTH, self._res_w.value)
                self._device.set(cv2.CAP_PROP_FRAME_HEIGHT, self._res_h.value)
                self._device.set(cv2.CAP_PROP_FPS, self._framerate.value)
                self._device.set(cv2.CAP_PROP_CONVERT_RGB, False)
                # Set auto exposure to false
                # TODO: this should be for watchtowers only
                if self._exposure_mode.value == 'sports':
                    msg = "Setting exposure to 'sports' mode."
                    self.loginfo(msg)
                    self._device.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
                    self._device.set(cv2.CAP_PROP_EXPOSURE, 40.0)
                # try getting a sample image
                retval, _ = self._device.read()
                if not retval:
                    msg = "Could not read image from camera"
                    self.logerr(msg)
                    raise RuntimeError(msg)
            except (Exception, RuntimeError):
                self.stop()
                msg = "Could not start camera"
                self.logerr(msg)
                raise RuntimeError(msg)
            # register self.close as cleanup function
            atexit.register(self.stop)

    def release(self, force: bool = False):
        if self._device is not None:
            self.loginfo('Releasing camera...')
            # noinspection PyBroadException
            try:
                self._device.release()
            except Exception:
                pass
            self.loginfo('Camera released.')
        self._device = None


if __name__ == '__main__':
    # initialize the node
    camera_node = RaspberryPi64Camera()
    camera_node.start()
    # keep the node alive
    rospy.spin()
