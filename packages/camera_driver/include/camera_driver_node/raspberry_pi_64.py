#!/usr/bin/env python3

import atexit
import subprocess
from typing import cast
import cv2
import asyncio
import argparse
import numpy as np


from camera_driver import CameraNodeAbs

class CameraNode(CameraNodeAbs):
    """
    Handles the imagery on a Raspberry Pi.
    """
    VIDEO_DEVICE = "/dev/video0"

    def __init__(self, config: str, sensor_name: str):
        # Initialize the DTROS parent class
        super(CameraNode, self).__init__(config, sensor_name)
        # prepare gstreamer pipeline
        self._device = None
        self.loginfo("[CameraNode]: Initialized.")

    async def worker(self):
        """
        Image capture procedure.

        Captures a frame from the /dev/video0 image sink and publishes it.
        """
        if self._device is None or not self._device.isOpened():
            self.logerr("Device was found closed")
            return
        # init queues
        await self.dtps_init_queues()
        # get first frame
        retval, image = self._device.read() if self._device else (False, None)

        # keep reading
        while not self.is_shutdown:
            if not retval:
                self.logerr("Could not read image from camera")
                await asyncio.sleep(1)
                continue
            if image is not None:
                # without HW acceleration, the image is returned as RGB, encode on CPU
                jpeg: bytes = cast(np.ndarray, image).tobytes()

                # publish
                await self.publish(jpeg)
            # return control to the event loop
            await asyncio.sleep(0.001)
            # grab next frame
            retval, image = self._device.read() if self._device else (False, None)
        self.loginfo("Camera worker stopped.")

    def setup(self):
        # setup camera
        cam_props = {
            "video_bitrate": 25000000,
        }
        for key in cam_props:
            subprocess.call(
                f"v4l2-ctl -d {CameraNode.VIDEO_DEVICE} -c {key}={str(cam_props[key])}", shell=True
            )
        # create VideoCapture object
        if self._device is None:
            self._device = cv2.VideoCapture()
        # open the device
        if not self._device.isOpened():
            try:
                self._device.open(CameraNode.VIDEO_DEVICE, cv2.CAP_V4L2)
                # make sure the device is open
                if not self._device.isOpened():
                    msg = "OpenCV cannot open camera"
                    self.logerr(msg)
                    raise RuntimeError(msg)
                # configure camera
                self._device.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                self._device.set(cv2.CAP_PROP_FRAME_WIDTH, self.configuration.res_w)
                self._device.set(cv2.CAP_PROP_FRAME_HEIGHT, self.configuration.res_h)
                self._device.set(cv2.CAP_PROP_ROLL, self.configuration.rotation)
                self._device.set(cv2.CAP_PROP_FPS, self.configuration.framerate)
                self._device.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
                # Set auto exposure to false
                if self.configuration.exposure_mode == "sports":
                    msg = "Setting exposure to 'sports' mode."
                    self.loginfo(msg)
                    self._device.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
                    self._device.set(cv2.CAP_PROP_EXPOSURE, self.configuration.exposure)
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

    def on_shutdown(self):
        if self._device is not None:
            self._device.release()
        self.loginfo("OpenCV device released.")

    def stop(self):
        """
        docstring
        """
        self.release()

    def release(self, force: bool = False):
        if self._device is not None:
            self.loginfo("Releasing camera...")
            # noinspection PyBroadException
            try:
                self._device.release()
            except Exception:
                pass
            self.loginfo("Camera released.")
        self._device = None


def main():
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("--sensor-name", type=str, required=True, help="Name of the sensor")
    parser.add_argument("--config", type=str, required=True, help="Name of the configuration")
    args: argparse.Namespace = parser.parse_args()
    # create node
    node: CameraNode = CameraNode(config=args.config, sensor_name=args.sensor_name)
    # launch the node
    node.spin()


if __name__ == "__main__":
    main()
