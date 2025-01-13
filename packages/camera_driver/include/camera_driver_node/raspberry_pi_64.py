#!/usr/bin/env python3

import argparse, asyncio, atexit, cv2, numpy
from camera_driver import CameraNodeAbs
from picamera2 import Picamera2


class CameraNode(CameraNodeAbs):
    """
    Handles the imagery on a Raspberry Pi.
    """
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
        if self._camera is None:
            self.logerr("Device was found closed")
            return
        # init queues
        await self.dtps_init_queues()
        # get first frame
        numpy_array = self._camera.capture_array() if self._camera else None
        if self.configuration.rotation:
            numpy_array = numpy.rot90(numpy_array)
        success, jpeg_encoded_numpy_array = cv2.imencode('.jpeg', numpy_array)
        # keep reading
        while not self.is_shutdown:
            if success is None:
                self.logerr("Could not read image from camera")
                await asyncio.sleep(1)
            else:
                jpeg: bytes = jpeg_encoded_numpy_array.tobytes()
                # publish
                await self.publish(jpeg)
            # return control to the event loop
            await asyncio.sleep(0.001)
            # grab next frame
            numpy_array = self._camera.capture_array() if self._camera else None
            if self.configuration.rotation:
                numpy_array = numpy.rot90(numpy_array)
            success, jpeg_encoded_numpy_array = cv2.imencode('.jpeg', numpy_array)
        self.loginfo("Camera worker stopped.")

    def setup(self):
        # setup camera
        self._camera = Picamera2()
        main = {
            "format": "RGB888"
        }
        controls = {
            "FrameRate": self.configuration.framerate
        }
        if self.configuration.exposure_mode == "sports":
            msg = "Setting exposure to 'sports' mode."
            self.loginfo(msg)
            controls["AeExposureMode"] = 1
        video_configuration = self._camera.create_video_configuration(main=main, controls=controls)
        self._camera.configure(video_configuration)
        # quality ranges from 0 (worst) to 95 (best), with 90 being the default
        self._camera.options["quality"] = 90
        self._camera.start()
        try:
            # try getting a sample image
            numpy_array = self._camera.capture_array()
            if numpy_array is None:
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
        super().on_shutdown()
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
