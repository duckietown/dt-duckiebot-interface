import asyncio, dataclasses, math, numpy, os, time
from abc import abstractmethod, ABCMeta
from dt_computer_vision.camera import CameraModel
from dt_node_utils import NodeType
from dt_node_utils.config import NodeConfiguration
from dt_node_utils.decorators import sidecar
from dt_node_utils.node import Node
from dtps import DTPSContext, PublisherInterface
from dtps_http import RawData
from dtps_http.structures import Bounds
from duckietown_messages.calibrations.camera_extrinsic import CameraExtrinsicCalibration
from duckietown_messages.calibrations.camera_intrinsic import CameraIntrinsicCalibration
from duckietown_messages.sensors.camera import Camera
from duckietown_messages.sensors.compressed_image import CompressedImage
from duckietown_messages.standard.header import Header
from hil_support.hil import HardwareInTheLoopSupport, HardwareInTheLoopSide
from kvstore_utils import KVStore
from turbojpeg import TurboJPEG
from typing import Optional, cast


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
    exposure_mode: str
    rotation: int = 0
    allow_partial_fov: Optional[bool] = None
    use_hw_acceleration: Optional[bool] = None
    exposure: Optional[int] = None


class CameraNodeAbs(Node, HardwareInTheLoopSupport, metaclass=ABCMeta):
    """
    Handles the imagery.

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
            description="Reads a stream of images from a camera and publishes the frames over DTPS"
        )
        HardwareInTheLoopSupport.__init__(self)
        self.robot_name = os.environ.get("VEHICLE_NAME")
        self.sensor_name: str = name
        self.frame_id = f"/{self.robot_name}/camera/{self.sensor_name}/optical_frame"
        # load configuration
        self.configuration: CameraNodeConfiguration = CameraNodeConfiguration.from_name(self.package, node_name, config)
        self.loginfo(f"Loaded configuration: {self.configuration.to_dict()}")
        # intrinsic calibration
        # TODO: take common part out
        intrinsics_calibration_folder = "/data/config/calibrations/camera_intrinsic/"
        # TODO: this needs to be adjusted location
        intrinsics_calibration_file = os.path.join(intrinsics_calibration_folder, f"{self.robot_name}.yaml")
        # find the intrinsics calibration file or use the default otherwise
        if not os.path.isfile(intrinsics_calibration_file):
            self.logwarn(f"Intrinsics calibration file not found: {intrinsics_calibration_file}.\nUsing default instead.")
            intrinsics_calibration_file = intrinsics_calibration_folder + "default.yaml"
            # shutdown if the default intrinsics calibration file is not found
            if not os.path.isfile(intrinsics_calibration_file):
                self.shutdown("Default intrinsics calibration file not found.\nShutting down...")
        # extrinsic calibration
        # TODO: take common part out
        extrinsics_calibration_folder = "/data/config/calibrations/camera_extrinsic/"
        # TODO: this needs to be adjusted location
        extrinsics_calibration_file = os.path.join(extrinsics_calibration_folder, f"{self.robot_name}.yaml")
        # find the extrinsics calibration file or use the default otherwise
        if not os.path.isfile(extrinsics_calibration_file):
            self.logwarn(f"Extrinsics calibration file not found: {extrinsics_calibration_file}.\nUsing default instead.")
            extrinsics_calibration_file = extrinsics_calibration_folder + "default.yaml"
            # shutdown if the default extrinsics calibration file is not found
            if not os.path.isfile(extrinsics_calibration_file):
                self.shutdown("Default extrinsics calibration file not found.\nShutting down...")
        # jpeg decoder
        self._jpeg: TurboJPEG = TurboJPEG()
        # queues
        self._has_published: bool = False
        self._jpeg_queue: Optional[DTPSContext] = None
        self._parameters_queue: Optional[DTPSContext] = None
        self._homography_queue: Optional[DTPSContext] = None
        self._info_queue: Optional[DTPSContext] = None
        # publishers
        self._jpeg_publisher: Optional[PublisherInterface] = None
        self._info_publisher: Optional[PublisherInterface] = None
        # data flow monitor
        self._last_image_published_time: float = time.time()
        # jpeg message
        self.jpeg_message: CompressedImage = CompressedImage(
            header=Header(
                frame=self.frame_id
            ),
            format="jpeg",
            data=b""
        )
        # ---
        self.loginfo("[CameraNodeAbs]: Initialized.")

    def _stale_stream_reset(self):
        self.shutdown("Data flow monitor has closed the node because of stale stream.")

    async def publish(self, jpeg: bytes):
        # do not publish if passthrough is active
        if self.hil_is_active:
            return
        # publish jpeg message
        self.jpeg_message.data = jpeg
        await self._jpeg_publisher.publish(self.jpeg_message.to_rawdata())
        # publish info message
        await self._info_publisher.publish(self.info_message_raw_data)
        self._last_image_published_time = time.time()
        if not self._has_published:
            self.loginfo("Published the first image")
            self.loginfo("Published camera info")
            self._has_published = True

    async def dtps_init_queues(self):
        await self.dtps_init(self.configuration)
        # create local queues
        out: DTPSContext = self.context / "out"
        self._jpeg_queue = await (out / "jpeg").queue_create(bounds=Bounds.max_length(3))
        self._parameters_queue = await (out / "parameters").queue_create()
        self._homography_queue = await (out / "homography").queue_create()
        self._info_queue = await (out / "info").queue_create()
        # create publishers
        self._jpeg_publisher = await self._jpeg_queue.publisher()
        self._info_publisher = await self._info_queue.publisher()
        # expose node to the switchboard
        await self.dtps_expose()
        # expose queues to the switchboard
        sensor: DTPSContext = self.switchboard / "sensor" / "camera" / self.sensor_name
        await (sensor / "jpeg").expose(self._jpeg_queue)
        await (sensor / "parameters").expose(self._parameters_queue)
        await (sensor / "homography").expose(self._homography_queue)
        await (sensor / "info").expose(self._info_queue)
        # open KVStore
        kvstore: KVStore = KVStore()
        await kvstore.init()
        # subscribe to the camera parameters
        await kvstore.subscribe("calibration/camera_intrinsic/current", self._on_new_intrinsic_calibration)
        # subscribe to the camera homography
        await kvstore.subscribe("calibration/camera_extrinsic/current", self._on_new_extrinsic_calibration)
        # initialize HIL support
        await self.init_hil_support(
            self.context,
            # source (this is the dynamic side, duckiematrix or nothing)
            src=None,
            src_path=["sensor", "camera", self.sensor_name],
            # destination (this is us, static)
            dst=self.context,
            dst_path=["out"],
            # paths to connect when a remote is set
            subpaths=["jpeg", "info"],
            # which side is the re-pluggable one
            side=HardwareInTheLoopSide.SOURCE
            # TODO: use transformations to set the frame in the message
        )

    async def _worker(self):
        """
        Begins the camera capturing.
        """
        self.loginfo("Setting up camera...")
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
            # ever able to use the camera and if we don't have an HIL passthrough active
            if self._has_published and i > failure_timeout and i % 5 == 0 and not self.hil_is_active:
                elapsed_since_last = time.time() - self._last_image_published_time
                # reset nvargus if no images were received within the last 10 secs
                if elapsed_since_last >= 10:
                    self.loginfo(
                        "[data-flow-monitor]: Detected a period of "
                        f"{int(elapsed_since_last)} seconds during which no "
                        "images were produced, restarting camera process."
                    )
                    self._stale_stream_reset()
            i += 1
            await asyncio.sleep(1)

    async def _on_new_extrinsic_calibration(self, rd: RawData):
        """
        Handles new extrinsic calibration data from the KVStore.
        """
        # load the new calibration
        try:
            calibration: dict = cast(dict, rd.get_as_native_object())
            if not isinstance(calibration, dict):
                raise ValueError(f"Expected a dictionary, received {type(calibration)} instead")
            # update the camera model
            self.camera_model.H = calibration["homography"]
        except Exception as e:
            self.logerr(f"Failed to process extrinsics calibration from KVStore:\n\nraw_data:\n{rd}\n\nexception:\n{e}")
            return
        # homography message
        homography_message: CameraExtrinsicCalibration = (
            CameraExtrinsicCalibration(homography=(
                self.camera_model.H if self.camera_model.H is not None else []
            ))
        )
        # publish homography message
        await self._homography_queue.publish(homography_message.to_rawdata())
        self.loginfo("Updated extrinsic camera calibration from KVStore.")

    async def _on_new_intrinsic_calibration(self, rd: RawData):
        """
        Handles new intrinsic calibration data from the KVStore.
        """
        # load the new calibration
        try:
            calibration: dict = cast(dict, rd.get_as_native_object())
            if not isinstance(calibration, dict):
                raise ValueError(f"Expected a dictionary, received {type(calibration)} instead")
            # update the camera model
            self.camera_model: CameraModel = CameraModel(
                width=calibration["image_width"],
                height=calibration["image_height"],
                K=calibration["camera_matrix"]["data"],
                D=calibration["distortion_coefficients"]["data"],
                P=calibration["projection_matrix"]["data"],
                R=calibration["rectification_matrix"]["data"]
            )
            if self.configuration.res_w != self.camera_model.width or self.configuration.res_h != self.camera_model.height:
                self.camera_model = self.scale_camera_model(self.camera_model, self.configuration)
        except Exception as e:
            self.logerr(f"Failed to process intrinsics calibration from KVStore:\n\nraw_data:\n{rd}\n\nexception:\n{e}")
            return
        # parameters message
        parameters_message: CameraIntrinsicCalibration = CameraIntrinsicCalibration(
            K=self.camera_model.K.flatten().tolist(),
            D=self.camera_model.D.flatten().tolist(),
            P=self.camera_model.P.flatten().tolist(),
            R=self.camera_model.R.flatten().tolist() if self.camera_model.R is not None else None
        )
        # info message
        info_message: Camera = Camera(
            header=Header(),
            name=self.sensor_name,
            type="camera",
            simulated=False,
            description="RGB8/JPEG Camera",
            frame_id=self.frame_id,
            frequency=self.configuration.framerate,
            maker=self.configuration.maker,
            model=self.configuration.model,
            width=self.camera_model.width,
            height=self.camera_model.height,
            fov=math.radians(self.configuration.fov)
        )
        self.info_message_raw_data = info_message.to_rawdata()
        # publish parameters message
        await self._parameters_queue.publish(parameters_message.to_rawdata())
        # publish info message
        await self._info_publisher.publish(self.info_message_raw_data)
        self.loginfo("Updated intrinsic camera calibration from KVStore.")
        self.loginfo("Published camera info")

    @abstractmethod
    def setup(self):
        raise NotImplementedError("Child classes should implement this method.")

    @staticmethod
    def scale_camera_model(camera_model: CameraModel, configuration: CameraNodeConfiguration):
        """
        Update the camera parameters based on the current resolution.

        The camera matrix, rectification matrix, and projection matrix depend on
        the resolution of the image.
        As the calibration has been done at a specific resolution, these matrices need
        to be adjusted if a different resolution is being used.
        """
        scale_width = float(configuration.res_w) / camera_model.width
        scale_height = float(configuration.res_h) / camera_model.height
        # adjust the K matrix
        scale_matrix = numpy.ones(9)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[4] *= scale_height
        scale_matrix[5] *= scale_height
        K = numpy.array(camera_model.K) * scale_matrix
        # adjust the P matrix
        scale_matrix = numpy.ones(12)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[5] *= scale_height
        scale_matrix[6] *= scale_height
        P = numpy.array(camera_model.P) * scale_matrix
        # create new camera model
        new_camera_model = CameraModel(
            width=configuration.res_w,
            height=configuration.res_h,
            K=K,
            D=camera_model.D,
            R=camera_model.R,
            P=P
        )
        return new_camera_model

    def on_shutdown(self):
        self.deinit_hil_support()
