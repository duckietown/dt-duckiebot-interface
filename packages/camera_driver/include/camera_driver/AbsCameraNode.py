import os
import time
import yaml
import copy
import rospy
import numpy as np
from threading import Thread

from abc import ABC, abstractmethod
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType


class AbsCameraNode(ABC, DTROS):
    """Handles the imagery.

    The node handles the image stream, initializing it, publishing frames
    according to the required frequency and stops it at shutdown.

    Note that only one instance of this class should be used at a time.
    If another node tries to start an instance while this node is running,
    it will likely fail with an `Out of resource` exception.

    The configuration parameters can be changed dynamically while the node is running via
    `rosparam set` commands.

    Configuration:
        ~framerate (:obj:`float`): The camera image acquisition framerate, default is 30.0 fps
        ~res_w (:obj:`int`): The desired width of the acquired image, default is 640px
        ~res_h (:obj:`int`): The desired height of the acquired image, default is 480px
        ~exposure_mode (:obj:`str`): PiCamera exposure mode, one of
            `these <https://picamera.readthedocs.io/en/latest/api_camera.html?highlight=sport#picamera.PiCamera.exposure_mode>`_, default is `sports`

    Publisher:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera images
        ~camera_info (:obj:`CameraInfo`): The camera parameters

    Service:
        ~set_camera_info:
            Saves a provided camera info
            to `/data/config/calibrations/camera_intrinsic/HOSTNAME.yaml`.

            input:
                camera_info (`CameraInfo`): The camera information to save

            outputs:
                success (`bool`): `True` if the call succeeded
                status_message (`str`): Used to give details about success

    """

    def __init__(self):
        # Initialize the DTROS parent class
        super(AbsCameraNode, self).__init__(
            node_name='camera',
            node_type=NodeType.DRIVER,
            help="Reads a stream of images from a camera and publishes the frames over ROS"
        )
        # Add the node parameters to the parameters dictionary and load their default values
        self._res_w = DTParam(
            '~res_w',
            param_type=ParamType.INT,
            help="Horizontal resolution (width) of the produced image frames."
        )
        self._res_h = DTParam(
            '~res_h',
            param_type=ParamType.INT,
            help="Vertical resolution (height) of the produced image frames."
        )
        self._framerate = DTParam(
            '~framerate',
            param_type=ParamType.INT,
            help="Framerate at which images frames are produced"
        )
        self._exposure_mode = DTParam(
            '~exposure_mode',
            param_type=ParamType.STRING,
            help="Exposure mode of the camera. Supported values are listed on "
                 "https://picamera.readthedocs.io/en/release-1.13/"
                 "api_camera.html#picamera.PiCamera.exposure_mode"
        )

        # define parameters
        self._framerate.register_update_callback(self.parameters_updated)
        self._res_w.register_update_callback(self.parameters_updated)
        self._res_h.register_update_callback(self.parameters_updated)
        self._exposure_mode.register_update_callback(self.parameters_updated)

        # intrinsic calibration
        self.cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        self.frame_id = rospy.get_namespace().rstrip('/') + '/camera_optical_frame'
        self.cali_file = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # locate calibration yaml file or use the default otherwise
        if not os.path.isfile(self.cali_file):
            self.logwarn("Calibration not found: %s.\n Using default instead." % self.cali_file)
            self.cali_file = (self.cali_file_folder + "default.yaml")

        # shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # load the calibration file
        self.original_camera_info = self.load_camera_info(self.cali_file)
        self.original_camera_info.header.frame_id = self.frame_id
        self.current_camera_info = copy.deepcopy(self.original_camera_info)
        self.update_camera_params()
        self.log("Using calibration file: %s" % self.cali_file)

        # create cv bridge
        self._bridge = CvBridge()

        # Setup publishers
        self._has_published = False
        self._is_stopped = False
        self._worker = None
        self.pub_img = rospy.Publisher(
            "~image/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="The stream of JPEG compressed images from the camera"
        )
        self.pub_camera_info = rospy.Publisher(
            "~camera_info",
            CameraInfo,
            queue_size=1,
            dt_topic_type=TopicType.DRIVER,
            dt_help="The stream of camera calibration information, the message content is fixed"
        )

        # Setup service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service(
            "~set_camera_info",
            SetCameraInfo,
            self.srv_set_camera_info_cb
        )
        # ---
        self.log("[AbsCameraNode]: Initialized.")

    @property
    def is_stopped(self):
        return self._is_stopped

    def parameters_updated(self):
        self.stop()
        self.update_camera_params()
        self.start()

    def publish(self, image_msg):
        # add time to messages
        stamp = rospy.Time.now()
        image_msg.header.stamp = stamp
        self.current_camera_info.header.stamp = stamp
        # update camera frame
        image_msg.header.frame_id = self.frame_id
        # publish image
        self.pub_img.publish(image_msg)
        # publish camera info
        self.pub_camera_info.publish(self.current_camera_info)
        # ---
        if not self._has_published:
            self.log("Published the first image.")
            self._has_published = True

    def start(self):
        """
        Begins the camera capturing.
        """
        self.log("Start capturing.")
        # ---
        try:
            self.setup()
            # run camera thread
            self._worker = Thread(target=self.run)
            self._worker.start()
        except StopIteration:
            self.log("Exception thrown.")

    def stop(self):
        self.loginfo('Stopping camera...')
        self._is_stopped = True
        # wait for the camera thread to finish
        if self._worker is not None:
            self._worker.join()
            time.sleep(1)
        self._worker = None
        # release resources
        self.release()
        time.sleep(1)
        self._is_stopped = False
        self.loginfo('Camera stopped.')

    @abstractmethod
    def setup(self):
        raise NotImplementedError('Child classes should implement this method.')

    @abstractmethod
    def release(self):
        raise NotImplementedError('Child classes should implement this method.')

    @abstractmethod
    def run(self):
        raise NotImplementedError('Child classes should implement this method.')

    def on_shutdown(self):
        self.stop()

    def srv_set_camera_info_cb(self, req):
        self.log("[srv_set_camera_info_cb] Callback!")
        filename = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.save_camera_info(req.camera_info, filename)
        response.status_message = "Write to %s" % filename
        return response

    def save_camera_info(self, camera_info_msg, filename):
        """Saves intrinsic calibration to file.

            Args:
                camera_info_msg (:obj:`CameraInfo`): Camera Info containg calibration
                filename (:obj:`str`): filename where to save calibration
        """
        # Convert camera_info_msg and save to a yaml file
        self.log("[save_camera_info] filename: %s" % filename)

        # Converted from camera_info_manager.py
        calib = {
            'image_width': camera_info_msg.width,
            'image_height': camera_info_msg.height,
            'camera_name': rospy.get_name().lstrip("/").split('/')[0],
            'distortion_model': camera_info_msg.distortion_model,
            'distortion_coefficients': {
                'data': camera_info_msg.D,
                'rows': 1,
                'cols': 5
            },
            'camera_matrix': {
                'data': camera_info_msg.K,
                'rows': 3,
                'cols': 3
            },
            'rectification_matrix': {
                'data': camera_info_msg.R,
                'rows': 3,
                'cols': 3
            },
            'projection_matrix': {
                'data': camera_info_msg.P,
                'rows': 3,
                'cols': 4
            }
        }
        self.log("[save_camera_info] calib %s" % calib)
        try:
            f = open(filename, 'w')
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False

    def update_camera_params(self):
        """ Update the camera parameters based on the current resolution.

        The camera matrix, rectification matrix, and projection matrix depend on
        the resolution of the image.
        As the calibration has been done at a specific resolution, these matrices need
        to be adjusted if a different resolution is being used.
        """
        scale_width = float(self._res_w.value) / self.original_camera_info.width
        scale_height = float(self._res_h.value) / self.original_camera_info.height

        scale_matrix = np.ones(9)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[4] *= scale_height
        scale_matrix[5] *= scale_height

        # adjust the camera matrix resolution
        self.current_camera_info.height = self._res_h.value
        self.current_camera_info.width = self._res_w.value

        # adjust the K matrix
        self.current_camera_info.K = np.array(self.original_camera_info.K) * scale_matrix

        # adjust the P matrix
        scale_matrix = np.ones(12)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[5] *= scale_height
        scale_matrix[6] *= scale_height
        self.current_camera_info.P = np.array(self.original_camera_info.P) * scale_matrix

    @staticmethod
    def load_camera_info(filename):
        """Loads the camera calibration files.

        Loads the intrinsic and extrinsic camera matrices.

        Args:
            filename (:obj:`str`): filename of calibration files.

        Returns:
            :obj:`CameraInfo`: a CameraInfo message object

        """
        with open(filename, 'r') as stream:
            calib_data = yaml.safe_load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info
