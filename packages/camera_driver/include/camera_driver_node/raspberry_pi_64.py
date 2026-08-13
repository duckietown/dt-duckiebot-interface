#!/usr/bin/env python3

import atexit
import subprocess
from typing import Any, Optional, Tuple, cast
import cv2
import asyncio
import argparse
import numpy as np


from camera_driver import CameraNodeAbs
from camera_driver.hw_jpeg_encoder import HardwareJpegEncoder

class CameraNode(CameraNodeAbs):
    """
    Handles the imagery on a Raspberry Pi.
    """
    VIDEO_DEVICE = "/dev/video0"
    JPEG_QUALITY = 90
    ROTATION_CODES = {
        90: cv2.ROTATE_90_CLOCKWISE,
        180: cv2.ROTATE_180,
        270: cv2.ROTATE_90_COUNTERCLOCKWISE,
    }

    def __init__(self, config: str, sensor_name: str):
        # Initialize the DTROS parent class
        super(CameraNode, self).__init__(config, sensor_name)
        self._device: Optional[Any] = None
        self._use_picamera2 = False
        self._hw_encoder: Optional[HardwareJpegEncoder] = None
        self._hw_encoder_disabled = False
        self.loginfo("[CameraNode]: Initialized.")

    def _get_capture_size(self) -> Tuple[int, int]:
        capture_width = self.configuration.res_w
        capture_height = self.configuration.res_h
        rotation = self.configuration.rotation % 360
        if rotation in (90, 270):
            capture_width = self.configuration.res_h
            capture_height = self.configuration.res_w
        return capture_width, capture_height

    def _rotate_image(self, image: np.ndarray) -> np.ndarray:
        rotation = self.configuration.rotation % 360
        code = self.ROTATION_CODES.get(rotation)
        if code is not None:
            return cv2.rotate(image, code)
        return image

    def _encode_jpeg(self, image: np.ndarray) -> Optional[bytes]:
        jpeg = self._encode_jpeg_hardware(image)
        if jpeg is not None:
            return jpeg
        encode_parameters = [cv2.IMWRITE_JPEG_QUALITY, self.JPEG_QUALITY]
        success, encoded = cv2.imencode(".jpg", image, encode_parameters)
        if not success:
            return None
        return encoded.tobytes()

    def _encode_jpeg_hardware(self, image: np.ndarray) -> Optional[bytes]:
        """Encode on the VideoCore JPEG block, or return None to fall back to software."""
        if self._hw_encoder_disabled:
            return None
        if self._hw_encoder is None:
            height, width = image.shape[:2]
            try:
                self._hw_encoder = HardwareJpegEncoder(width, height, quality=self.JPEG_QUALITY)
            except Exception as exc:
                self._hw_encoder_disabled = True
                self.logwarn(f"Hardware JPEG encoder unavailable, using software: {exc}")
                return None
            self.loginfo(f"JPEG encoding on hardware ({self._hw_encoder.device})")
        try:
            return self._hw_encoder.encode(image)
        except Exception as exc:
            self.logwarn(f"Hardware JPEG encode failed, falling back to software: {exc}")
            self._release_hw_encoder()
            self._hw_encoder_disabled = True
            return None

    def _release_hw_encoder(self):
        if self._hw_encoder is not None:
            try:
                self._hw_encoder.close()
            except Exception as exc:
                self.logwarn(f"Failed to close hardware JPEG encoder: {exc}")
            self._hw_encoder = None

    def _capture_jpeg(self) -> Optional[bytes]:
        if self._device is None:
            return None
        if self._use_picamera2:
            image = self._device.capture_array()
            if image is None:
                return None
        else:
            video_capture = cast(cv2.VideoCapture, self._device)
            success, image = video_capture.read()
            if not success or image is None:
                return None
        image_array = cast(np.ndarray, image)
        rotated_image = self._rotate_image(image_array)
        return self._encode_jpeg(rotated_image)

    def _set_optional_v4l2_control(self, key: str, value: int):
        command = ["v4l2-ctl", "-d", self.VIDEO_DEVICE, "-c", f"{key}={value}"]
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        if result.returncode == 0:
            return
        reason = result.stderr.strip()
        if not reason:
            reason = result.stdout.strip()
        if reason:
            self.logwarn(f"Skipping unsupported camera control '{key}': {reason}")

    def _try_picamera2(self) -> bool:
        try:
            from picamera2 import Picamera2
        except ImportError:
            self.logwarn("picamera2 not installed; skipping libcamera path")
            return False
        camera = None
        capture_width, capture_height = self._get_capture_size()
        try:
            camera = Picamera2()
            video_configuration = camera.create_video_configuration(
                main={"format": "RGB888", "size": (capture_width, capture_height)},
                controls={"FrameRate": float(self.configuration.framerate)},
            )
            camera.configure(video_configuration)
            if self.configuration.exposure_mode == "sports":
                self.loginfo("Setting exposure to 'sports' mode.")
                camera.set_controls({"AeExposureMode": 1})
            camera.start()
            first_frame = camera.capture_array()
            if first_frame is None:
                raise RuntimeError("picamera2 opened but first frame was empty")
            self._device = camera
            self._use_picamera2 = True
            self.loginfo("Camera opened via picamera2 (libcamera)")
            return True
        except Exception as exc:
            if camera is not None:
                try:
                    camera.stop()
                except Exception as stop_exc:
                    self.logwarn(f"Failed to stop picamera2 during cleanup: {stop_exc}")
                try:
                    camera.close()
                except Exception as close_exc:
                    self.logwarn(f"Failed to close picamera2 during cleanup: {close_exc}")
            self.logwarn(f"picamera2 failed: {exc}")
            return False

    def _open_v4l2(self):
        self._set_optional_v4l2_control("video_bitrate", 25000000)
        if self._device is None:
            self._device = cv2.VideoCapture()
        video_capture = cast(cv2.VideoCapture, self._device)
        if video_capture.isOpened():
            return
        capture_width, capture_height = self._get_capture_size()
        try:
            video_capture.open(self.VIDEO_DEVICE, cv2.CAP_V4L2)
            if not video_capture.isOpened():
                raise RuntimeError("OpenCV cannot open camera")
            video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, capture_width)
            video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_height)
            video_capture.set(cv2.CAP_PROP_FPS, self.configuration.framerate)
            if self.configuration.exposure_mode == "sports":
                self.loginfo("Setting exposure to 'sports' mode.")
                video_capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
                if self.configuration.exposure is not None:
                    video_capture.set(cv2.CAP_PROP_EXPOSURE, self.configuration.exposure)
            self._use_picamera2 = False
            if self._capture_jpeg() is None:
                raise RuntimeError("Could not read image from camera")
            self.loginfo(f"Camera opened via V4L2 ({self.VIDEO_DEVICE})")
        except Exception as exc:
            self.stop()
            raise RuntimeError(f"Could not start camera: {exc}")

    async def worker(self):
        """
        Image capture procedure.

        Captures a frame from the /dev/video0 image sink and publishes it.
        """
        if self._device is None:
            self.logerr("Device was found closed")
            return
        # init queues
        await self.dtps_init_queues()

        # keep reading
        while not self.is_shutdown:
            jpeg = self._capture_jpeg()
            if jpeg is None:
                self.logerr("Could not read image from camera")
                await asyncio.sleep(1)
                continue
            await self.publish(jpeg)
            # return control to the event loop
            await asyncio.sleep(0.001)
        self.loginfo("Camera worker stopped.")

    def setup(self):
        if self._try_picamera2():
            atexit.register(self.stop)
            return
        try:
            self._open_v4l2()
        except RuntimeError as exc:
            self.logerr(str(exc))
            raise
        atexit.register(self.stop)

    def on_shutdown(self):
        super().on_shutdown()
        self.release()

    def stop(self):
        """
        docstring
        """
        self.release()

    def release(self, force: bool = False):
        self._release_hw_encoder()
        if self._device is not None:
            self.loginfo("Releasing camera...")
            if self._use_picamera2:
                try:
                    self._device.stop()
                except Exception as exc:
                    self.logerr(f"Failed to stop picamera2 device: {exc}")
                try:
                    self._device.close()
                except Exception as exc:
                    self.logerr(f"Failed to close picamera2 device: {exc}")
            else:
                try:
                    self._device.release()
                except Exception as exc:
                    self.logerr(f"Failed to release V4L2 device: {exc}")
            self.loginfo("Camera released.")
        self._device = None
        self._use_picamera2 = False


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
