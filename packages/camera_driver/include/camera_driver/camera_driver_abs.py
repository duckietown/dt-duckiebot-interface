import asyncio
import copy
import dataclasses
import math
import os
import time
from abc import abstractmethod, ABCMeta
from typing import Optional

import numpy as np
import yaml
from turbojpeg import TurboJPEG

from dt_computer_vision.camera import CameraModel
from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.decorators import sidecar
from dt_node_utils.node import Node
from dtps import DTPSContext
from dtps_http.structures import Bounds
from duckietown_messages.calibrations.camera_extrinsic import CameraExtrinsicCalibration
from duckietown_messages.calibrations.camera_intrinsic import CameraIntrinsicCalibration
from duckietown_messages.geometry_2d.homography import Homography
from duckietown_messages.sensors.camera import Camera
from duckietown_messages.sensors.compressed_image import CompressedImage
from duckietown_messages.standard.header import Header

from dtps_utils.passthrough import DTPSPassthrough


@dataclasses.dataclass
class CameraNodeConfiguration(NodeConfiguration):
    """
    framerate (:obj:`float`): The camera image acquisition framerate
    res_w (:obj:`int`): The desired width of the acquired image
    res_h (:obj:`int`): The desired height of the acquired image
    exposure_mode (:obj:`str`): exposure mode, one of
        `these <https://picamera.readthedocs.io/en/latest/api_camera.html?highlight=sport#picamera.PiCamera.exposure_mode>`_, default is `sports`
    """
    maker: str
    model: str
    framerate: int
    fov: int
    res_w: int
    res_h: int
    rotation: int = 0
    exposure_mode: str
    rotation: int
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

    def __init__(self, config: str, name: str):
        node_name: str = f"camera_driver_{name}"
        super().__init__(
            name=node_name,
            kind=NodeType.DRIVER,
            description="Reads a stream of images from a camera and publishes the frames over DTPS",
        )
        self.sensor_name: str = name

        # load configuration
        self.configuration: CameraNodeConfiguration = CameraNodeConfiguration.from_name(self.package, node_name, config)
        self.loginfo(f"Loaded configuration: {self.configuration.to_dict()}")

        # intrinsic calibration
        # TODO: take common part out
        self.cali_file_folder = "/data/config/calibrations/camera_intrinsic/"
        self.frame_id = f"/{self._robot_name}/camera/{self.sensor_name}/optical_frame"
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
        self._homographies_queue: Optional[DTPSContext] = None
        self._info_queue: Optional[DTPSContext] = None
        # data flow monitor
        self._last_image_published_time: float = time.time()
        # passthrough context
        self._passthrough: Optional[DTPSPassthrough] = None
        # ---
        self.loginfo("[CameraNodeAbs]: Initialized.")

    def _stale_stream_reset(self):
        self.shutdown("Data flow monitor has closed the node because of stale stream.")

    async def publish(self, jpeg: bytes):
        # do not publish if passthrough is active
        if self._passthrough.is_active:
            return
        # ---
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
            # publish camera intrinsics
            msg: CameraIntrinsicCalibration = CameraIntrinsicCalibration(
                K=self.camera_model.K.tolist(),
                D=self.camera_model.D.tolist(),
                P=self.camera_model.P.tolist(),
                R=self.camera_model.R.tolist() if self.camera_model.R is not None else None,
            )
            await self._parameters_queue.publish(msg.to_rawdata())
            self.loginfo("Published camera intrinsics")
            # publish camera extrinsics
            msg: CameraExtrinsicCalibration = CameraExtrinsicCalibration(
                homographies={
                    f"/{self._robot_name}/base_footprint": Homography(data=self.camera_model.H.tolist())
                } if self.camera_model.H is not None else {},
            )
            await self._homographies_queue.publish(msg.to_rawdata())
            self.loginfo(f"Published {len(msg.homographies)} known camera homographies")
            # publish camera info
            msg: Camera = Camera(
                # -- base
                header=Header(),
                # -- sensor
                name=self.sensor_name,
                type="camera",
                simulated=False,
                description="RGB8/JPEG Camera",
                frame_id=self.frame_id,
                frequency=self.configuration.framerate,
                maker=self.configuration.maker,
                model=self.configuration.model,
                # -- camera
                width=self.camera_model.width,
                height=self.camera_model.height,
                fov=math.radians(self.configuration.fov),
            )
            await self._info_queue.publish(msg.to_rawdata())
            self.loginfo("Published camera info")
            # ---
            self.loginfo("Published the first image")
            self._has_published = True

    async def dtps_init_queues(self):
        await self.dtps_init(self.configuration)
        # create local queues
        out: DTPSContext = self.context / "out"
        self._jpeg_queue = await (out / "jpeg").queue_create(bounds=Bounds.max_length(3))
        self._parameters_queue = await (out / "parameters").queue_create()
        self._homographies_queue = await (out / "homographies").queue_create()
        self._info_queue = await (out / "info").queue_create()
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        sensor: DTPSContext = self.switchboard / "sensor" / "camera" / self.sensor_name
        await (sensor / "jpeg").expose(self._jpeg_queue)
        await (sensor / "parameters").expose(self._parameters_queue)
        await (sensor / "homographies").expose(self._homographies_queue)
        await (sensor / "info").expose(self._info_queue)
        # create passthrough
        self._passthrough = DTPSPassthrough(self.context, None, sensor, ["jpeg"])
        await self._passthrough.astart()

    async def _worker(self):
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

    @sidecar
    async def flow_monitor(self):
        i = 0
        failure_timeout = 20
        while not self.is_shutdown:
            # do nothing for the first `failure_timeout` seconds, then check every 5 seconds, but only if we were
            # ever able to use the camera and if we don't have a passthrough active
            if self._has_published and i > failure_timeout and i % 5 == 0 and not self._passthrough.is_active:
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
            await asyncio.sleep(1)

    @abstractmethod
    def setup(self):
        raise NotImplementedError("Child classes should implement this method.")

    # TODO: we should use a shared dict with YAML file backend with default fallback and a schema instead of a
    #  service-like function
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
