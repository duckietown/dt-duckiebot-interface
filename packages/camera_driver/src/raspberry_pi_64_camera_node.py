#!/usr/bin/env python3
import time
from typing import cast

import cv2
import rospy
import atexit
import numpy as np

from camera_driver import AbsCameraNode

from duckietown.utils.image.ros import rgb_to_compressed_imgmsg

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


        self._last_time = 0

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

                print(1.0 / (time.time() - self._last_time))
                self._last_time = time.time()

                if True:

                    image = cast(np.ndarray, image)
                    if image.shape[0] == 1:
                        # image is already JPEG encoded
                        image_msg = CompressedImage()
                        image_msg.header.stamp = rospy.Time.now()
                        image_msg.format = "jpeg"
                        image_msg.data = image.tobytes()
                    else:
                        # image is a BGR array, encode first
                        image_msg = rgb_to_compressed_imgmsg(image, encoding='jpeg')
                    # publish the compressed image
                    self.publish(image_msg)
            # grab next frame
            retval, image = self._device.read() if self._device else (False, None)
        self.loginfo('Camera worker stopped.')

    def setup(self):
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
                # print(self._res_w.value)
                # print(self._res_h.value)

                #
                # $ v4l2-ctl -d /dev/video0 --list-formats
                # ioctl: VIDIOC_ENUM_FMT
                # 	Type: Video Capture
                #
                # 	[0]: 'YU12' (Planar YUV 4:2:0)
                # 	[1]: 'YUYV' (YUYV 4:2:2)
                # 	[2]: 'RGB3' (24-bit RGB 8-8-8)
                # 	[3]: 'JPEG' (JFIF JPEG, compressed)
                # 	[4]: 'H264' (H.264, compressed)
                # 	[5]: 'MJPG' (Motion-JPEG, compressed)
                # 	[6]: 'YVYU' (YVYU 4:2:2)
                # 	[7]: 'VYUY' (VYUY 4:2:2)
                # 	[8]: 'UYVY' (UYVY 4:2:2)
                # 	[9]: 'NV12' (Y/CbCr 4:2:0)
                # 	[10]: 'BGR3' (24-bit BGR 8-8-8)
                # 	[11]: 'YV12' (Planar YVU 4:2:0)
                # 	[12]: 'NV21' (Y/CrCb 4:2:0)
                # 	[13]: 'RX24' (32-bit XBGR 8-8-8-8)
                #
                #


                # self._device.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))


                self._device.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self._device.set(cv2.CAP_PROP_FRAME_WIDTH, 1296)
                self._device.set(cv2.CAP_PROP_FRAME_HEIGHT, 972)
                self._device.set(cv2.CAP_PROP_FPS, 30)
                # self._device.set(cv2.CAP_PROP_CONVERT_RGB, True)
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

    def release(self):
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
