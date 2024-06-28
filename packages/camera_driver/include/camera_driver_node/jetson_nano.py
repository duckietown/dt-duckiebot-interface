#!/usr/bin/env python3

import cv2
import time
import psutil
import asyncio
import argparse
import numpy as np

from typing import Tuple, cast
from collections import namedtuple

from camera_driver import CameraNodeAbs


CameraMode = namedtuple("CameraMode", "id width height fps fov")


class CameraNode(CameraNodeAbs):
    """
    Handles the imagery on a Jetson Nano.
    """

    # each mode defines [width, height, fps]
    CAMERA_MODES = [
        CameraMode(0, 3264, 2464, 21, "full"),
        CameraMode(1, 3264, 1848, 28, "partial"),
        CameraMode(2, 1920, 1080, 30, "partial"),
        CameraMode(3, 1640, 1232, 30, "full"),
        CameraMode(4, 1280, 720, 60, "partial"),
        CameraMode(5, 1280, 720, 120, "partial"),
    ]
    # exposure time range (ns)
    EXPOSURE_TIMERANGES = {"sports": [100000, 80000000], "night": [100000, 1000000000]}
    DEFAULT_EXPOSURE_MODE = "sports"

    def __init__(self, config: str, sensor_name: str):
        # Initialize the DTROS parent class
        super(CameraNode, self).__init__(config, sensor_name)
        # prepare gstreamer pipeline
        self._device = None
        # ---
        self.loginfo("[CameraNode]: Initialized.")

    def _stale_stream_reset(self):
        # find PID of the nvargus process
        killed = False
        for proc in psutil.process_iter():
            try:
                # get process name and pid from process object
                process_cmdline = proc.cmdline()
                if len(process_cmdline) <= 0:
                    continue
                process_bin = process_cmdline[0]
                # if it is not the one we are looking for, skip
                if not process_bin.startswith("/usr/sbin/nvargus-daemon"):
                    continue
                # this is the one we are looking for
                self.loginfo(
                    f"[data-flow-monitor]: Process 'nvargus-daemon' found "
                    f"with PID #{proc.pid}"
                )
                # - kill nvargus
                self.loginfo(f"[data-flow-monitor]: Killing nvargus.")
                proc.kill()
                time.sleep(1)
                # - exit camera node, will be respawned by the external process manager
                self.shutdown("Data flow monitor has closed the node")
                time.sleep(1)
                exit(1)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        if not killed:
            self.loginfo("[data-flow-monitor]: Process 'nvargus-daemon' not found.")

    async def worker(self):
        """
        Image capture procedure.

        Captures frames from the camera and publishes them.
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
            # do not read the sensor if the passthrough is active
            if self.hil_is_active:
                await asyncio.sleep(1)
                continue

            if not retval:
                self.logerr("Could not read image from camera")
                await asyncio.sleep(1)
                continue
            if image is not None:
                if self.configuration.use_hw_acceleration:
                    # with HW acceleration, the NVJPG Engine module is used to encode RGB -> JPEG
                    jpeg: bytes = cast(np.ndarray, image).tobytes()
                else:
                    # without HW acceleration, the image is returned as RGB, encode on CPU
                    jpeg: bytes = np.uint8(image)
                # publish
                await self.publish(jpeg)
            # return control to the event loop
            await asyncio.sleep(0.001)
            # grab next frame
            retval, image = self._device.read() if self._device else (False, None)
        self.loginfo("Camera worker stopped.")

    def setup(self):
        if self._device is None:
            self._device = cv2.VideoCapture()
        # check if the device is opened
        if self._device is None:
            msg = "OpenCV cannot open empty VideoCapture"
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
            except (Exception, RuntimeError) as e:
                msg = f"Could not start camera: {e}"
                self.logerr(msg)
                raise RuntimeError(msg)

    def gst_pipeline_string(self) -> str:
        res_w, res_h, fps = self.configuration.res_w, self.configuration.res_h, self.configuration.framerate
        fov = ("full", "partial") if self.configuration.allow_partial_fov else ("full",)
        # find best mode
        camera_mode = self.get_mode(res_w, res_h, fps, fov)
        # cap frequency
        if fps > camera_mode.fps:
            self.logwarn(
                "Camera framerate({}fps) too high for the camera mode (max: {}fps), "
                "capping at {}fps.".format(fps, camera_mode.fps, camera_mode.fps)
            )
            fps = camera_mode.fps
        # get exposure time
        exposure_time = self.EXPOSURE_TIMERANGES.get(
            self.configuration.exposure_mode, self.EXPOSURE_TIMERANGES[self.DEFAULT_EXPOSURE_MODE]
        )
        # compile gst pipeline
        if self.configuration.use_hw_acceleration:
            gst_pipeline = """ \
                nvarguscamerasrc \
                sensor-mode={} exposuretimerange="{} {}" ! \
                video/x-raw(memory:NVMM), width={}, height={}, format=NV12, framerate={}/1 ! \
                nvjpegenc ! \
                appsink \
            """.format(
                camera_mode.id, *exposure_time, self.configuration.res_w, self.configuration.res_h, fps
            )
        else:
            gst_pipeline = """ \
                nvarguscamerasrc \
                sensor-mode={} exposuretimerange="{} {}" ! \
                video/x-raw(memory:NVMM), width={}, height={}, format=NV12, framerate={}/1 ! \
                nvvidconv ! 
                video/x-raw, format=BGRx ! 
                videoconvert ! \
                appsink \
            """.format(
                camera_mode.id, *exposure_time, self.configuration.res_w, self.configuration.res_h, fps
            )
        # ---
        self.logdebug("Using GST pipeline: `{}`".format(gst_pipeline))
        return gst_pipeline

    def get_mode(self, width: int, height: int, fps: int, fov: Tuple[str]) -> CameraMode:
        candidates = {
            m
            for m in self.CAMERA_MODES
            if m.width >= width and m.height >= height and m.fps >= fps and m.fov in fov
        }.union({self.CAMERA_MODES[0]})
        return sorted(candidates, key=lambda m: m.id)[-1]

    def on_shutdown(self):
        if self._device is not None:
            self._device.release()
        self.loginfo("OpenCV device released.")


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
