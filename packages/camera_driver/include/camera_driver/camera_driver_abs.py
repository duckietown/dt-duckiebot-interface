import asyncio
import dataclasses
import os
import time
from threading import Thread
from typing import Optional

import yaml
import copy
import numpy as np

from abc import abstractmethod, ABCMeta

from turbojpeg import TurboJPEG

from dtps import DTPSContext

from dt_computer_vision.camera import CameraModel
from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.node import Node
from duckietown_messages.sensors.camera import Camera
from duckietown_messages.sensors.compressed_image import CompressedImage
from duckietown_messages.standard.header import Header


@dataclasses.dataclass
class CameraNodeConfiguration(NodeConfiguration):
    """
    framerate (:obj:`float`): The camera image acquisition framerate
    res_w (:obj:`int`): The desired width of the acquired image
    res_h (:obj:`int`): The desired height of the acquired image
    exposure_mode (:obj:`str`): exposure mode, one of
        `these <https://picamera.readthedocs.io/en/latest/api_camera.html?highlight=sport#picamera.PiCamera.exposure_mode>`_, default is `sports`
    """
    framerate: int
    res_w: int
    res_h: int
    exposure_mode: str
    allow_partial_fov: Optional[bool] = None
    use_hw_acceleration: Optional[bool] = None
    exposure: Optional[int] = None


class CameraNodeAbs(Node, metaclass=ABCMeta):
    """Handles the imagery.

    The node handles the image stream, initializing it, publishing frames
    according to the required frequency and stops it at shutdown.

    Note that only one instance of this class should be used at a time.
    If another node tries to start an instance while this node is running,
    it will likely fail with an `Out of resource` exception.

    """

    def __init__(self, config: str):
        super().__init__(
            name=f"camera",
            kind=NodeType.DRIVER,
            description="Reads a stream of images from a camera and publishes the frames over DTPS",
        )

        # load configuration
        self.configuration: CameraNodeConfiguration = CameraNodeConfiguration.from_name(self.package, config)

        # intrinsic calibration
        # TODO: take common part out
        self.cali_file_folder = "/data/config/calibrations/camera_intrinsic/"
        self.frame_id = f"/{self._robot_name}/camera_optical_frame"
        # TODO: this needs to be adjusted location
        self.cali_file = os.path.join(self.cali_file_folder, f"{self._robot_name}.yaml")

        # locate calibration yaml file or use the default otherwise
        if not os.path.isfile(self.cali_file):
            self.logwarn("Calibration not found: %s.\n Using default instead." % self.cali_file)
            self.cali_file = self.cali_file_folder + "default.yaml"

        # shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            self.shutdown("Found no calibration file ... aborting")

        # load the calibration file
        self._original_camera_model: CameraModel = self.load_camera_model(self.cali_file)
        self.camera_model: CameraModel = self.compute_camera_model(self._original_camera_model, self.configuration)
        self.loginfo("Using calibration file: %s" % self.cali_file)

        # jpeg decoder
        self._jpeg: TurboJPEG = TurboJPEG()

        # publishers
        self._has_published: bool = False
        self._jpeg_queue: Optional[DTPSContext] = None
        self._parameters_queue: Optional[DTPSContext] = None
        # data flow monitor
        self._last_image_published_time: float = 0
        # TODO: use sidecar instead of a thread
        self._flow_monitor = Thread(target=self._flow_monitor_fcn, daemon=True)
        self._flow_monitor.start()
        # ---
        self.loginfo("[CameraNodeAbs]: Initialized.")

    def _stale_stream_reset(self):
        self.shutdown("Data flow monitor has closed the node because of stale stream.")

    def _flow_monitor_fcn(self):
        i = 0
        failure_timeout = 20
        while not self.is_shutdown:
            # do nothing for the first `sleep_until` seconds, then check every 5 seconds
            if (self._has_published or i > failure_timeout) and i % 5 == 0:
                elapsed_since_last = time.time() - self._last_image_published_time
                # reset nvargus if no images were received within the last 10 secs
                if elapsed_since_last >= 10:
                    self.loginfo(
                        f"[data-flow-monitor]: Detected a period of "
                        f"{int(elapsed_since_last)} seconds during which no "
                        f"images were produced, restarting camera process."
                    )
                    self._stale_stream_reset()
            # ---
            i += 1
            time.sleep(1)

    async def publish(self, jpeg: bytes):
        msg: CompressedImage = CompressedImage(
            header=Header(
                frame=self.frame_id,
            ),
            format="jpeg",
            data=jpeg,
        )
        # publish the compressed image
        await self._jpeg_queue.publish(msg.to_rawdata())
        self._last_image_published_time = time.time()
        # ---
        if not self._has_published:
            # publish camera model
            msg: Camera = Camera.from_camera_model(self.camera_model)
            await self._parameters_queue.publish(msg.to_rawdata())
            self.loginfo("Published camera model")
            # ---
            self.loginfo("Published the first image")
            self._has_published = True

    async def dtps_init_queues(self):
        await self.dtps_init(self.configuration)
        # create sensor queue
        self._jpeg_queue = await (self.context / "out" / "jpeg").queue_create()
        # create model queue
        self._parameters_queue = await (self.context / "out" / "parameters").queue_create()
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        await (self.switchboard / "sensors" / "camera" / "jpeg").expose(self._jpeg_queue)
        await (self.switchboard / "sensors" / "camera" / "parameters").expose(self._parameters_queue)

    async def __worker(self):
        """
        Begins the camera capturing.
        """
        self.loginfo("Setting up camera...")
        # ---
        try:
            # setup camera
            try:
                self.setup()
            except RuntimeError as e:
                self.shutdown(f"Failed to setup camera: {e}")
                return
            # run camera worker
            self.loginfo("Start capturing.")
            await self.worker()
        except StopIteration:
            self.logerr("Exception thrown.")

    @abstractmethod
    def setup(self):
        raise NotImplementedError("Child classes should implement this method.")

    def spin(self):
        try:
            asyncio.run(self.__worker())
        except (KeyboardInterrupt, asyncio.CancelledError):
            self.__on_shutdown()

    def save_camera_info(self, camera_model: CameraModel, filename):
        """Saves intrinsic calibration to file.

        Args:
            camera_model (:obj:`CameraModel`): Camera model
            filename (:obj:`str`): filename where to save the camera model
        """
        # Convert camera_info_msg and save to a yaml file
        self.loginfo("[save_camera_info] filename: %s" % filename)

        # serialize to yaml
        # TODO: we should version this format
        calib = {
            "image_width": camera_model.width,
            "image_height": camera_model.height,
            "camera_name": "front",
            "distortion_coefficients": {"data": camera_model.D, "rows": 1, "cols": 5},
            "camera_matrix": {"data": camera_model.K, "rows": 3, "cols": 3},
            "rectification_matrix": {"data": camera_model.R, "rows": 3, "cols": 3},
            "projection_matrix": {"data": camera_model.P, "rows": 3, "cols": 4},
        }
        self.loginfo("[save_camera_info] calib %s" % calib)
        try:
            f = open(filename, "w")
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False

    @staticmethod
    def compute_camera_model(original: CameraModel, cfg: CameraNodeConfiguration):
        """
        Update the camera parameters based on the current resolution.

        The camera matrix, rectification matrix, and projection matrix depend on
        the resolution of the image.
        As the calibration has been done at a specific resolution, these matrices need
        to be adjusted if a different resolution is being used.
        """
        scale_width = float(cfg.res_w) / original.width
        scale_height = float(cfg.res_h) / original.height

        scale_matrix = np.ones(9)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[4] *= scale_height
        scale_matrix[5] *= scale_height

        # adjust the camera matrix resolution
        new_height = cfg.res_h
        new_width = cfg.res_w

        # adjust the K matrix
        new_k = np.array(original.K) * scale_matrix

        # adjust the P matrix
        scale_matrix = np.ones(12)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[5] *= scale_height
        scale_matrix[6] *= scale_height
        new_p = np.array(original.P) * scale_matrix

        # create new camera model
        return CameraModel(
            width=new_width,
            height=new_height,
            K=new_k,
            D=copy.deepcopy(original.D),
            R=copy.deepcopy(original.R),
            P=new_p,
        )

    @staticmethod
    def load_camera_model(filename: str) -> CameraModel:
        """Loads the camera calibration files.

        Loads the intrinsic camera matrices.

        Args:
            filename (:obj:`str`): filename of calibration files.

        Returns:
            :obj:`CameraModel`: a CameraModel message object

        """
        with open(filename, "r") as stream:
            calibration = yaml.safe_load(stream)
        camera = CameraModel(
            width=calibration["image_width"],
            height=calibration["image_height"],
            K=calibration["camera_matrix"]["data"],
            D=calibration["distortion_coefficients"]["data"],
            R=calibration["rectification_matrix"]["data"],
            P=calibration["projection_matrix"]["data"],
        )
        return camera
