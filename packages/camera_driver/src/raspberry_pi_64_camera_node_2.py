#!/usr/bin/env python3

import cv2
import rospy
import atexit
import numpy as np

from typing import Tuple, cast
from collections import namedtuple

from camera_driver import AbsCameraNode

from sensor_msgs.msg import CompressedImage

from duckietown.utils.image.ros import rgb_to_compressed_imgmsg

CameraMode = namedtuple('CameraMode', 'id width height fps fov')


class RaspberryPi64CameraNode(AbsCameraNode):
    """
    Handles the imagery on a Jetson Nano.
    """

    # each mode defines [width, height, fps]
    CAMERA_MODES = [
        # All Modes as returned by `gst-inspect-1.0 rpicamsrc`
        #
        #       (0): automatic        - Automatic
        #       (1): 1920x1080        - 1920x1080 16:9 1-30fps
        #       (2): 2592x1944-fast   - 2592x1944 4:3 1-15fps / 3240x2464 15fps w/ v.2 board
        #       (3): 2592x1944-slow   - 2592x1944 4:3 0.1666-1fps / 3240x2464 15fps w/ v.2 board
        #       (4): 1296x972         - 1296x972 4:3 1-42fps
        #       (5): 1296x730         - 1296x730 16:9 1-49fps
        #       (6): 640x480-slow     - 640x480 4:3 42.1-60fps
        #       (7): 640x480-fast     - 640x480 4:3 60.1-90fps
        #
        CameraMode(1, 1920, 1080, 30, 'partial'),
        CameraMode(2, 2592, 1944, 15, 'full'),
        CameraMode(3, 2592, 1944, 1, 'full'),
        CameraMode(4, 1296, 972, 42, 'full'),
        CameraMode(5, 1296, 730, 49, 'full'),
        CameraMode(6, 640, 480, 60, 'full'),
        CameraMode(7, 640, 480, 90, 'full'),
    ]
    DEFAULT_EXPOSURE_MODE = "sports"

    def __init__(self):
        # Initialize the DTROS parent class
        super(RaspberryPi64CameraNode, self).__init__()
        # parameters
        self._allow_partial_fov = rospy.get_param('~allow_partial_fov', False)
        # prepare gstreamer pipeline
        self._device = None
        # ---
        self.log("[RaspberryPi64CameraNode]: Initialized.")

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
                if image.shape[0] == 1:
                    # image is already JPEG encoded
                    image = cast(np.ndarray, image)
                    image_msg = CompressedImage()
                    image_msg.header.stamp = rospy.Time.now()
                    image_msg.format = "jpeg"
                    image_msg.data = image.tobytes()
                else:
                    # image is a BGR array, encode first
                    # image_msg = rgb_to_compressed_imgmsg(image, encoding='jpeg')
                    pass
                # publish the compressed image
                self.publish(image_msg)
            # grab next frame
            retval, image = self._device.read() if self._device else (False, None)
        self.loginfo('Camera worker stopped.')

    def setup(self):
        if self._device is None:
            self._device = cv2.VideoCapture()
        # check if the device is opened
        if self._device is None:
            msg = "OpenCV cannot open gstreamer resource"
            self.logerr(msg)
            raise RuntimeError(msg)
        # open the device
        if not self._device.isOpened():
            try:
                self._device.open(self.gst_pipeline_string(), cv2.CAP_GSTREAMER)

                # self._device.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'JPEG'))
                # self._device.set(cv2.CAP_PROP_FRAME_WIDTH, 1296)
                # self._device.set(cv2.CAP_PROP_FRAME_HEIGHT, 972)
                # self._device.set(cv2.CAP_PROP_FPS, 30)
                # self._device.set(cv2.CAP_PROP_CONVERT_RGB, False)

                # make sure the device is open
                if not self._device.isOpened():
                    msg = "OpenCV cannot open gstreamer resource"
                    self.logerr(msg)
                    raise RuntimeError(msg)
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
            self.loginfo('Releasing GST pipeline...')
            try:
                self._device.release()
            except Exception:
                pass
            self.loginfo('GST pipeline released.')
        self._device = None

    def gst_pipeline_string(self):
        res_w, res_h, fps = self._res_w.value, self._res_h.value, self._framerate.value
        fov = ('full', 'partial') if self._allow_partial_fov else ('full',)
        # find best mode
        camera_mode = self.get_mode(res_w, res_h, fps, fov)
        self.loginfo(f"Best camera mode based on requirements "
                     f"(w:{res_w}, h:{res_h}, hz:{fps}, fov:{fov}) is #{camera_mode.id}: "
                     f"{str(camera_mode)}")
        # cap frequency
        if fps > camera_mode.fps:
            self.logwarn("Camera framerate({}fps) too high for the camera mode (max: {}fps), "
                         "capping at {}fps.".format(fps, camera_mode.fps, camera_mode.fps))
            fps = camera_mode.fps
        # compile gst pipeline
        gst_pipeline = """ \
            rpicamsrc sensor-mode=1296x972 ! \
            image/jpeg, framerate={}/1 ! \
            queue ! \
            appsink \
        """.format(
            fps
        )


        # gst_pipeline = "v4l2src device=/dev/video0 io-mode=2 ! " \
        #                "video/x-h264,stream-format=byte-stream ! decodebin ! videorate ! video/x-raw ! videoconvert ! " \
        #                "appsink"

        # gst_pipeline = "v4l2src device=/dev/video0 io-mode=2 ! image/jpeg ! appsink"

        # ---
        self.logdebug("Using GST pipeline: `{}`".format(gst_pipeline))
        return gst_pipeline

    def get_mode(self, width: int, height: int, fps: int, fov: Tuple[str]) -> CameraMode:
        candidates = {
            m for m in self.CAMERA_MODES
            if m.width >= width and m.height >= height and m.fps >= fps and m.fov in fov
        }.union({self.CAMERA_MODES[0]})
        return sorted(candidates, key=lambda m: m.id)[-1]


if __name__ == '__main__':
    # initialize the node
    camera_node = RaspberryPi64CameraNode()
    camera_node.start()
    # keep the node alive
    rospy.spin()
