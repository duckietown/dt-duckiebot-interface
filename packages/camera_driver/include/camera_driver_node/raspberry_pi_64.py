#!/usr/bin/env python3
import argparse
import asyncio
import atexit
from typing import Optional, Union

import cv2
import numpy as np

from camera_driver import CameraNodeAbs

class CameraNode(CameraNodeAbs):
    """
    Handles the imagery on a Raspberry Pi 4/5 running Bookworm (64-bit).

    Primary path: picamera2 / libcamera (CSI camera modules, e.g. IMX219,
    OV5647).  Falls back to OpenCV + V4L2 for USB cameras or any device
    that presents a /dev/video* V4L2 node without libcamera support.

    The node publishes the same DTPS queue layout as the Jetson Nano driver
    (sensor/camera/<name>/{jpeg,info,parameters,homography}) so that the
    dt-ros2-interface camera bridge can be reused without modification.
    """

    # Rotation (degrees CW) → numpy rot90 counter-clockwise steps
    _ROTATION_K = {0: 0, 90: 3, 180: 2, 270: 1}

    VIDEO_DEVICE = "/dev/video0"
    JPEG_QUALITY = 90

    def __init__(self, config: str, sensor_name: str):
        super(CameraNode, self).__init__(config, sensor_name)
        self._camera: Optional[Union["Picamera2", cv2.VideoCapture]] = None
        self._use_picamera2: bool = False
        self.loginfo("[CameraNode]: Initialized.")

    async def worker(self):
        """
        Main capture loop.  Grabs frames as fast as the camera produces them
        and hands each one to the base-class publish() as JPEG bytes.
        """
        if self._camera is None:
            self.logerr("Device was found closed")
            return
        # init queues
        await self.dtps_init_queues()
        while not self.is_shutdown:
            jpeg = self._capture_jpeg()
            if jpeg is None:
                self.logerr("Could not capture frame from camera")
                await asyncio.sleep(1)
                continue
            await self.publish(jpeg)
            # yield to the event loop between frames
            await asyncio.sleep(0)
        self.loginfo("Camera worker stopped.")

    def setup(self):
        """
        Open the camera.  picamera2 (libcamera) is tried first; V4L2 is the
        fallback.  Raises RuntimeError when neither interface works.
        """
        if not self._try_picamera2():
            self._try_v4l2()

    # ------------------------------------------------------------------
    # picamera2 (libcamera) path — preferred on Bookworm RPi 4/5
    # ------------------------------------------------------------------

    def _try_picamera2(self) -> bool:
        """
        Attempt to open the camera via picamera2/libcamera.

        Returns True on success, False if picamera2 is not installed or the
        camera cannot be opened.
        """
        try:
            from picamera2 import Picamera2  # type: ignore[import]
        except ImportError:
            self.logwarn("picamera2 not installed; skipping libcamera path.")
            return False
        try:
            cam = Picamera2()
            cfg = cam.create_video_configuration(
                main={
                    "format": "RGB888",
                    "size": (self.configuration.res_w, self.configuration.res_h),
                },
                controls={
                    "FrameRate": float(self.configuration.framerate),
                },
            )
            cam.configure(cfg)
            # JPEG quality hint used by picamera2's still-capture helpers
            cam.options["quality"] = self.JPEG_QUALITY
            if self.configuration.exposure_mode == "sports":
                self.loginfo("Setting AeExposureMode=1 (sports).")
                cam.set_controls({"AeExposureMode": 1})
            cam.start()
            # smoke-test: grab one frame to confirm the pipeline works
            frame = cam.capture_array()
            if frame is None:
                cam.stop()
                cam.close()
                self.logwarn("picamera2 opened but could not capture a frame.")
                return False
            self._camera = cam
            self._use_picamera2 = True
            atexit.register(self.stop)
            self.loginfo("Camera opened via picamera2 (libcamera).")
            return True
        except Exception as e:
            self.logwarn(f"picamera2 failed to open camera: {e}")
            return False

    # ------------------------------------------------------------------
    # V4L2 / OpenCV fallback path — USB cameras and legacy setups
    # ------------------------------------------------------------------

    def _try_v4l2(self):
        """
        Open /dev/video0 via OpenCV + V4L2 with MJPEG output.
        Raises RuntimeError when the device cannot be opened.
        """
        cap = cv2.VideoCapture()
        try:
            cap.open(CameraNode.VIDEO_DEVICE, cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError("OpenCV cannot open camera via V4L2")
            # request MJPEG from the camera (avoids in-kernel JPEG decode/re-encode)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.configuration.res_w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.configuration.res_h)
            cap.set(cv2.CAP_PROP_FPS, self.configuration.framerate)
            # keep raw MJPEG bytes; do NOT let OpenCV decode and re-encode
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
            if self.configuration.exposure_mode == "sports":
                self.loginfo("Setting auto-exposure to 'sports' mode.")
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
                if self.configuration.exposure is not None:
                    cap.set(cv2.CAP_PROP_EXPOSURE, self.configuration.exposure)
            retval, _ = cap.read()
            if not retval:
                raise RuntimeError("Could not read image from camera via V4L2")
            self._camera = cap
            self._use_picamera2 = False
            atexit.register(self.stop)
            self.loginfo("Camera opened via V4L2 (OpenCV).")
        except Exception as e:
            cap.release()
            msg = f"Could not start camera: {e}"
            self.logerr(msg)
            raise RuntimeError(msg)

    # ------------------------------------------------------------------
    # Frame capture helpers
    # ------------------------------------------------------------------

    def _capture_jpeg(self) -> Optional[bytes]:
        """Return the current frame as JPEG bytes, or None on failure."""
        if self._use_picamera2:
            return self._capture_jpeg_picamera2()
        return self._capture_jpeg_v4l2()

    def _capture_jpeg_picamera2(self) -> Optional[bytes]:
        frame = self._camera.capture_array()
        if frame is None:
            return None
        # apply clockwise rotation via counter-clockwise numpy rot90
        k = self._ROTATION_K.get(self.configuration.rotation, 0)
        if k:
            frame = np.rot90(frame, k=k)
        ok, buf = cv2.imencode(
            ".jpg", frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.JPEG_QUALITY],
        )
        return buf.tobytes() if ok else None

    def _capture_jpeg_v4l2(self) -> Optional[bytes]:
        retval, image = self._camera.read()
        if not retval or image is None:
            return None
        # With FOURCC=MJPG and CONVERT_RGB=0, OpenCV returns the raw MJPEG
        # buffer as a 1-D numpy array; .tobytes() gives the JPEG stream.
        return image.tobytes()

    # ------------------------------------------------------------------
    # Lifecycle hooks
    # ------------------------------------------------------------------

    def on_shutdown(self):
        super().on_shutdown()
        self.release()

    def stop(self):
        self.release()

    def release(self, force: bool = False):
        if self._camera is not None:
            self.loginfo("Releasing camera...")
            try:
                if self._use_picamera2:
                    self._camera.stop()
                    self._camera.close()
                else:
                    self._camera.release()
            except Exception:
                pass
            self.loginfo("Camera released.")
        self._camera = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sensor-name", type=str, required=True)
    parser.add_argument("--config", type=str, required=True)
    args = parser.parse_args()
    node = CameraNode(config=args.config, sensor_name=args.sensor_name)
    node.spin()


if __name__ == "__main__":
    main()
