#!/usr/bin/env python3

import cv2
import rospy
import atexit
import numpy as np

from camera_driver import AbsCameraNode


class JetsonNanoCameraNode(AbsCameraNode):
    """
    Handles the imagery on a Jetson Nano.
    """

    # each mode defines [width, height, fps]
    CAMERA_MODES = {
        0: [3264, 2464, 21],
        1: [3264, 1848, 28],
        2: [1920, 1080, 30],
        3: [1280, 720, 60],
        4: [1280, 720, 120],
    }
    # exposure time range (ns)
    EXPOSURE_TIMERANGES = {
        "sports": [100000, 80000000],
        "night":  [100000, 1000000000]
    }
    DEFAULT_EXPOSURE_MODE = "sports"

    def __init__(self):
        # Initialize the DTROS parent class
        super(JetsonNanoCameraNode, self).__init__()
        # prepare gstreamer pipeline
        self._camera_mode = 0
        self._device = None
        # ---
        self.log("[JetsonNanoCameraNode]: Initialized.")

    def run(self):
        """ Image capture procedure.
        
            Captures a frame from the /dev/video2 image sink and publishes it.
        """
        if self._device is None or not self._device.isOpened():
            self.logerr('Device was found closed')
            return
        # get first frame
        retval, image = self._device.read() if self._device else (False, None)
        # keep reading
        while (not self.is_stopped) and (not self.is_shutdown) and retval:
            # generate the compressed image
            if image is not None:
                image = np.uint8(image)
            image_msg = self._bridge.cv2_to_compressed_imgmsg(image, dst_format='jpeg')
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
        # find best mode
        capture_w, capture_h, fps = \
            self.get_mode(self._res_w.value, self._res_h.value, self._framerate.value)
        # cap frequency
        if self._framerate.value > fps:
            self.logwarn("Camera framerate({}fps) too high for the camera mode (max: {}fps), "
                         "capping at {}fps.".format(self._framerate.value, fps, fps))
        else:
            fps = self._framerate.value
        # get exposure time
        exposure_time = self.EXPOSURE_TIMERANGES.get(
            self._exposure_mode,
            self.EXPOSURE_TIMERANGES[self.DEFAULT_EXPOSURE_MODE]
        )
        # compile gst pipeline
        gst_pipeline = """ \
            nvarguscamerasrc \
            sensor-mode={} exposuretimerange="{} {}" ! \
            video/x-raw(memory:NVMM), \
            width={}, height={}, format=(string)NV12, framerate={}/1 ! \
            nvvidconv ! video/x-raw, format=(string)BGRx ! \
            videoconvert ! video/x-raw, format=BGR ! \
            videoscale ! video/x-raw, width={}, height={} ! \
            appsink \
        """.format(
            self._camera_mode,
            *exposure_time,
            capture_w,
            capture_h,
            fps,
            self._res_w.value,
            self._res_h.value
        )
        self.logdebug("Using GST pipeline: `{}`".format(gst_pipeline))
        return gst_pipeline

    def get_mode(self, width, height, fps):
        candidates = {
            i for (i, (_w, _h, _f)) in enumerate(self.CAMERA_MODES)
            if _w >= width and _h >= height and _f >= fps
        }.union({0})
        return self.CAMERA_MODES[sorted(candidates)[-1]]


if __name__ == '__main__':
    # initialize the node
    camera_node = JetsonNanoCameraNode()
    camera_node.start()
    # keep the node alive
    rospy.spin()
