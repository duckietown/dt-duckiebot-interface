#!/usr/bin/env python3

import cv2
import rospy
import atexit

from camera_driver import AbsCameraNode

from duckietown.utils.image.ros import rgb_to_compressed_imgmsg


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
                # noinspection PyTypeChecker
                image_msg = rgb_to_compressed_imgmsg(image, encoding='jpeg')
                print(image.shape)
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
                self._device.open(RaspberryPi64Camera.VIDEO_DEVICE, cv2.CAP_V4L)
                # make sure the device is open
                if not self._device.isOpened():
                    msg = "OpenCV cannot open camera"
                    self.logerr(msg)
                    raise RuntimeError(msg)
                # configure camera
                # print(self._res_w.value)
                # print(self._res_h.value)
                # self._device.set(cv2.CAP_PROP_FRAME_WIDTH, self._res_w.value)
                # self._device.set(cv2.CAP_PROP_FRAME_HEIGHT, self._res_h.value)
                # try getting a sample image
                retval, _ = self._device.read()


                print(_.shape)


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
