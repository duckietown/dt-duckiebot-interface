#!/usr/bin/env python
import io
import os
import thread
import yaml
import rospy
import copy
import numpy as np

from duckietown import DTROS
from picamera import PiCamera
from sensor_msgs.msg import CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

class CameraNode(DTROS):
    """Handles the imagery.

    The node handles the image stream, initializing it, publishing frames
    according to the required frequency and stops it at shutdown.
    `Picamera <https://picamera.readthedocs.io/>`_ is used for handling the image stream.

    Note that only one :obj:`PiCamera` object should be used at a time. If another node tries to start
    an instance while this node is running, it will likely fail with an `Out of resource` exception.

    The configuration parameters can be changed dynamically while the node is running via `rosparam set` commands.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~framerate (:obj:`float`): The camera image acquisition framerate, default is 30.0 fps
        ~res_w (:obj:`int`): The desired width of the acquired image, default is 640px
        ~res_h (:obj:`int`): The desired height of the acquired image, default is 480px
        ~exposure_mode (:obj:`str`): PiCamera exposure mode, one of `these <https://picamera.readthedocs.io/en/latest/api_camera.html?highlight=sport#picamera.PiCamera.exposure_mode>`_, default is `sports`

    Publisher:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera images

    Service:
        ~set_camera_info:
            Saves a provided camera info to `/data/config/calibrations/camera_intrinsic/HOSTNAME.yaml`.

            input:
                camera_info (`CameraInfo`): The camera information to save

            outputs:
                success (`bool`): `True` if the call succeeded
                status_message (`str`): Used to give details about success

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(CameraNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~framerate'] = None
        self.parameters['~res_w'] = None
        self.parameters['~res_h'] = None
        self.parameters['~exposure_mode'] = None
        self.updateParameters()

        # Setup PiCamera
        self.image_msg = CompressedImage()
        self.camera = PiCamera()
        self.camera.framerate = self.parameters['~framerate']
        self.camera.resolution = (self.parameters['~res_w'], self.parameters['~res_h'])
        self.camera.exposure_mode = self.parameters['~exposure_mode']


        # For intrinsic calibration
        self.cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'
        self.cali_file = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(self.cali_file):
            self.log("Can't find calibration file: %s.\n Using default calibration instead."
                          % self.cali_file, 'warn')
            self.cali_file = (self.cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Load the calibration file
        self.original_camera_info = self.loadCameraInfo(self.cali_file)
        self.original_camera_info.header.frame_id = self.frame_id
        self.current_camera_info = copy.deepcopy(self.original_camera_info)
        self.updateCameraParameters()
        self.log("Using calibration file: %s" % self.cali_file)

        # Setup publishers
        self.has_published = False
        # self.pub_img = rospy.Publisher("~image/compressed", CompressedImage, queue_size=1)
        # self.pub_camera_info = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)
        self.pub_img = self.publisher("~image/compressed", CompressedImage, queue_size=1)
        self.pub_camera_info = self.publisher("~camera_info", CameraInfo, queue_size=1)

        # Setup service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service("~set_camera_info",
                                                 SetCameraInfo,
                                                 self.cbSrvSetCameraInfo)
        self.stream = io.BytesIO()

        self.log("Initialized.")

    def startCapturing(self):
        """Initialize and closes image stream.

            Begin the camera capturing. When the node shutdowns, closes the
            image stream. If it detects StopIteration exception from the `grabAndPublish`
            generator due to parameter change, will update the parameters and
            restart the image capturing.
        """
        self.log("Start capturing.")
        while not self.is_shutdown and not rospy.is_shutdown():
            gen = self.grabAndPublish(self.stream)
            try:
                self.camera.capture_sequence(gen,
                                             'jpeg',
                                             use_video_port=True,
                                             splitter_port=0)
            except StopIteration:
                pass

            # Update the camera parameters
            self.camera.framerate = self.parameters['~framerate']
            self.camera.resolution = (self.parameters['~res_w'], self.parameters['~res_h'])
            self.camera.exposure_mode = self.parameters['~exposure_mode']

            # Update the camera info parameters
            self.updateCameraParameters()

            self.parametersChanged = False
            self.log("Parameters updated.")

        self.camera.close()
        self.log("Capture Ended.")

    def grabAndPublish(self, stream):
        """Captures a frame from stream and publishes it.

            If the stream is stable (no parameter updates or shutdowns),
            grabs a frame, creates the image message and publishes it. If there is a paramter change,
            it does raises StopIteration exception which is caught by `startCapturing`. It updates the
            camera parameters and restarts the recording.

            Args:
                stream (:obj:`BytesIO`): imagery stream
                publisher (:obj:`Publisher`): publisher of topic
        """
        while not (self.parametersChanged or self.is_shutdown or rospy.is_shutdown()):
            yield stream
            # Construct image_msg
            # Grab image from stream
            stamp = rospy.Time.now()
            stream.seek(0)
            stream_data = stream.getvalue()

            # Generate and publish the compressed image
            image_msg = CompressedImage()
            image_msg.format = "jpeg"
            image_msg.data = stream_data
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            self.pub_img.publish(image_msg)

            # Publish the CameraInfo message
            self.current_camera_info.header.stamp = stamp
            self.pub_camera_info.publish(self.current_camera_info)

            # Clear stream
            stream.seek(0)
            stream.truncate()

            if not self.has_published:
                self.log("Published the first image.")
                self.has_published = True

            rospy.sleep(rospy.Duration.from_sec(0.001))

    def cbSrvSetCameraInfo(self, req):
        self.log("[cbSrvSetCameraInfo] Callback!")
        filename = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.saveCameraInfo(req.camera_info, filename)
        response.status_message = "Write to %s" % filename
        return response

    def saveCameraInfo(self, camera_info_msg, filename):
        """Saves intrinsic calibration to file.

            Args:
                camera_info_msg (:obj:`CameraInfo`): Camera Info containg calibration
                filename (:obj:`str`): filename where to save calibration
        """
        # Convert camera_info_msg and save to a yaml file
        self.log("[saveCameraInfo] filename: %s" % (filename))

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
                 'image_height': camera_info_msg.height,
                 'camera_name': rospy.get_name().strip("/"),  # TODO check this
                 'distortion_model': camera_info_msg.distortion_model,
                 'distortion_coefficients': {'data': camera_info_msg.D,
                                             'rows': 1,
                                             'cols': 5},
                 'camera_matrix': {'data': camera_info_msg.K,
                                   'rows': 3,
                                   'cols': 3},
                 'rectification_matrix': {'data': camera_info_msg.R,
                                          'rows': 3,
                                          'cols': 3},
                 'projection_matrix': {'data': camera_info_msg.P,
                                       'rows': 3,
                                       'cols': 4}}

        self.log("[saveCameraInfo] calib %s" % (calib))

        try:
            f = open(filename, 'w')
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False

    def updateCameraParameters(self):
        """ Update the camera parameters based on the current resolution.

        The camera matrix, rectification matrix, and projection matrix depend on the resolution
        of the image. As the calibration has been done at a specific resolution, these matrices need
        to be adjusted if a different resolution is being used.

        TODO: Test that this really works.
        """

        scale_width = float(self.parameters['~res_w']) / self.original_camera_info.width
        scale_height = float(self.parameters['~res_h']) / self.original_camera_info.height

        scale_matrix = np.ones(9)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[4] *= scale_height
        scale_matrix[5] *= scale_height

        # Adjust the camera matrix resolution
        self.current_camera_info.height = self.parameters['~res_h']
        self.current_camera_info.width = self.parameters['~res_w']

        # Adjust the K matrix
        self.current_camera_info.K = np.array(self.original_camera_info.K) * scale_matrix

        # Adjust the P matrix (done by Rohit)
        scale_matrix = np.ones(12)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[5] *= scale_height
        scale_matrix[6] *= scale_height
        self.current_camera_info.P = np.array(self.original_camera_info.P) * scale_matrix


    def loadCameraInfo(self, filename):
        """Loads the camera calibration files.

        Loads the intrinsic and extrinsic camera matrices.

        Args:
            filename (:obj:`str`): filename of calibration files.

        Returns:
            :obj:`CameraInfo`: a CameraInfo message object

        """
        stream = file(filename, 'r')
        calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info


if __name__ == '__main__':
    # Initialize the node
    camera_node = CameraNode(node_name='camera')
    # Start the image capturing in a separate thread
    thread.start_new_thread(camera_node.startCapturing, ())
    # Keep it spinning to keep the node alive
    rospy.spin()
