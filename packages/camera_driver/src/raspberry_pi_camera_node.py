#!/usr/bin/env python3

import io
import rospy

from picamera import PiCamera
from sensor_msgs.msg import CompressedImage

from camera_driver import AbsCameraNode


class RaspberryPiCameraNode(AbsCameraNode):
    """
    Handles the imagery on a Raspberry Pi.
    """

    def __init__(self):
        # Initialize the DTROS parent class
        super(RaspberryPiCameraNode, self).__init__()
        # prepare camera device
        self._device = None
        self._stream = io.BytesIO()
        # ---
        self.log("[RaspberryPiCameraNode]: Initialized.")

    def _process_frame(self, stream):
        # keep reading
        while (not self.is_stopped) and (not self.is_shutdown):
            # expose buffer for the camera to populate with a new frame
            yield stream
            # move to the beginning of the buffer and get its value in bytes
            stream.seek(0)
            stream_data = stream.getvalue()
            # make a ROS message out of the raw data
            image_msg = CompressedImage()
            # update message content
            image_msg.format = "jpeg"
            image_msg.data = stream_data
            # publish message
            self.publish(image_msg)
            # clear buffer
            stream.seek(0)
            stream.truncate()
        self.loginfo('Camera worker stopped.')

    def run(self):
        """ Image capture procedure.

            Captures a frame from the CSI camera and publishes it.
        """
        if self._device is None or self._device.closed:
            self.logerr('Device was found closed')
            return
        # create infinite iterator
        processor = self._process_frame(self._stream)
        # start processing data from camera
        try:
            self._device.capture_sequence(
                processor,
                'jpeg',
                use_video_port=True,
                splitter_port=0
            )
        except StopIteration:
            pass

    def setup(self):
        if self._device is None:
            self._device = PiCamera()
            self._device.framerate = self._framerate.value
            self._device.resolution = (self._res_w.value, self._res_h.value)
            self._device.exposure_mode = self._exposure_mode.value

    def release(self):
        if self._device is not None:
            self.loginfo('Releasing CSI camera...')
            try:
                self._device.close()
            except Exception:
                pass
            self.loginfo('CSI camera released.')
        self._device = None


if __name__ == '__main__':
    # initialize the node
    camera_node = RaspberryPiCameraNode()
    camera_node.start()
    # keep the node alive
    rospy.spin()
