#!/usr/bin/env python3

import io
import os
import yaml
import copy
import numpy as np
from threading import Thread

from dt_device_utils import get_device_hardware_brand, DeviceHardwareBrand
ROBOT_HARDWARE = get_device_hardware_brand()

# for Jetson Nano
if ROBOT_HARDWARE == DeviceHardwareBrand.JETSON_NANO:
    import cv2
    import atexit
    from cv_bridge import CvBridge

# for RPi
elif ROBOT_HARDWARE == DeviceHardwareBrand.RASPBERRY_PI:
    from picamera import PiCamera

else:
    raise Exception("Undefined Hardware!")

import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

from duckietown.dtros import DTROS, NodeType, TopicType


class CameraNode(DTROS):
    """Handles the imagery.

    The node handles the image stream, initializing it, publishing frames
    according to the required frequency and stops it at shutdown.
    `Picamera <https://picamera.readthedocs.io/>`_ is used for handling the image stream on the Raspberry Pi.
    Opencv is used for handling the image stream on the Jetson Nano.

    Note that only one :obj:`PiCamera` object should be used at a time.
    If another node tries to start an instance while this node is running,
    it will likely fail with an `Out of resource` exception.

    The configuration parameters can be changed dynamically while the node is running via
    `rosparam set` commands.

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
            Saves a provided camera info
            to `/data/config/calibrations/camera_intrinsic/HOSTNAME.yaml`.

            input:
                camera_info (`CameraInfo`): The camera information to save

            outputs:
                success (`bool`): `True` if the call succeeded
                status_message (`str`): Used to give details about success

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(CameraNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.DRIVER,
            help="Reads a stream of images from a Pi Camera and publishes the frames over ROS"
        )

        # Add the node parameters to the parameters dictionary and load their default values
        self._framerate = rospy.get_param(
            '~framerate',
            dt_help="Framerate at which images frames are produced"
        )
        self._res_w = rospy.get_param(
            '~res_w',
            dt_help="Horizontal resolution (width) of the produced image frames."
        )
        self._res_h = rospy.get_param(
            '~res_h',
            dt_help="Vertical resolution (height) of the produced image frames."
        )
        self._exposure_mode = rospy.get_param(
            '~exposure_mode',
            dt_help="Exposure mode of the camera. Supported values are listed on "
                    "https://picamera.readthedocs.io/en/release-1.13/"
                    "api_camera.html#picamera.PiCamera.exposure_mode"
        )

        # Jetson Nano Camera initialization
        if ROBOT_HARDWARE == DeviceHardwareBrand.JETSON_NANO:

            self.value = np.empty((480, 640, 3), dtype=np.uint8)
            
            try:
                self.cap = cv2.VideoCapture(2)
                
                if not self.cap.isOpened():
                    print("cv2 can not open resource")

                re, image = self.cap.read()

                if not re:
                    raise RuntimeError("Could not read image from camera.")
                
                self.value = image
                self.start()

            except:
                self.stop()
                raise RuntimeError("Could not start camera.")

            atexit.register(self.stop)

            # Set camera parameters
            self.cap.set(cv2.CAP_PROP_FPS, self._framerate)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._res_w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self._res_h)
            #self.cap.set(CV_CAP_PROP_MODE, self.parameters['~exposure_mode'])

        # RPi Camera initialization
        else:
            # Setup PiCamera
            self.image_msg = CompressedImage()
            self.camera = PiCamera()
            self.camera.framerate = self._framerate
            self.camera.resolution = (self._res_w, self._res_h)
            self.camera.exposure_mode = self._exposure_mode

            self.stream = io.BytesIO()
        
        # For intrinsic calibration
        self.cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'
        self.cali_file = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(self.cali_file):
            self.logwarn("Calibration not found: %s.\n Using default instead." % self.cali_file)
            self.cali_file = (self.cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Load the calibration file
        self.original_camera_info = self.load_camera_info(self.cali_file)
        self.original_camera_info.header.frame_id = self.frame_id
        self.current_camera_info = copy.deepcopy(self.original_camera_info)
        self.update_camera_params()
        self.log("Using calibration file: %s" % self.cali_file)

        # Setup publishers
        self.has_published = False
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

        self.log("Initialized.")

    def start_capturing(self):
        """Initialize and closes image stream.

            Begin the camera capturing. When the node shutdowns, closes the
            image stream. If it detects StopIteration exception from the `grab_and_publish`
            generator due to parameter change, will update the parameters and
            restart the image capturing.
        """
        self.log("Start capturing.")

        # Jetson Nano capture procedure
        if ROBOT_HARDWARE == DeviceHardwareBrand.JETSON_NANO:
            while not self.is_shutdown:
                try:
                    self.start()
                    self.grab_and_publish_jetson()

                except StopIteration:
                    self.log("Exception thrown.")

        # RPi capture procedure
        else:
            while not self.is_shutdown:
                gen = self.grab_and_publish_rpi(self.stream)

                try:
                    self.camera.capture_sequence(
                        gen,
                        'jpeg',
                        use_video_port=True,
                        splitter_port=0
                    )
                except StopIteration:
                    pass

            self.camera.close()

        self.log("Capture Ended.")

    def grab_and_publish_jetson(self):

        """ Image capture procedure for the Jetson Nano
        
            Captures a frame from the /dev/video2 image sink and publishes it.

            If the stream is stable (no shutdowns),
            grabs a frame, creates the image message and publishes it.
            Will not detect parameter change.

        """
        re, image = self.cap.read()
        bridge = CvBridge()

        while re:
            re, image = self.cap.read()

            #Generate the compressed image
            if image is not None:
                image = np.uint8(image)
                
            stamp = rospy.Time.now()
            image_message = bridge.cv2_to_compressed_imgmsg(image, dst_format='jpeg')
                
            #not sure if this actualy does something/ gets published
            image_message.header.stamp = stamp
            image_message.header.frame_id = self.frame_id

            #Publish the compressed image
            self.pub_img.publish(image_message)

            # Publish the CameraInfo message 
            self.current_camera_info.header.stamp = stamp
            self.pub_camera_info.publish(self.current_camera_info)

            if not self.has_published:
                self.log("Published the first image.")
                self.has_published = True

            rospy.sleep(rospy.Duration.from_sec(0.001))
        
    def grab_and_publish_rpi(self, stream):
        """ Image capture procedure for the Raspberry Pi

            Captures a frame from stream and publishes it.
            If the stream is stable (no parameter updates or shutdowns),
            grabs a frame, creates the image message and publishes it.
            If there is a paramter change, it does raises StopIteration exception
            which is caught by `start_capturing`.
            It updates the camera parameters and restarts the recording.

            Args:
                stream (:obj:`BytesIO`): imagery stream
        """
        while not self.is_shutdown:
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

    def start(self):
        if not self.cap.isOpened():
            self.cap.open(2)

    def stop(self):
        if hasattr(self, 'cap'):
            self.cap.release()

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
            'camera_name': rospy.get_name().strip("/"),  # TODO check this
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

        TODO: Test that this really works.
        """

        scale_width = float(self._res_w) / self.original_camera_info.width
        scale_height = float(self._res_h) / self.original_camera_info.height

        scale_matrix = np.ones(9)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[4] *= scale_height
        scale_matrix[5] *= scale_height

        # Adjust the camera matrix resolution
        self.current_camera_info.height = self._res_h
        self.current_camera_info.width = self._res_w

        # Adjust the K matrix
        self.current_camera_info.K = np.array(self.original_camera_info.K) * scale_matrix

        # Adjust the P matrix (done by Rohit)
        scale_matrix = np.ones(12)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[5] *= scale_height
        scale_matrix[6] *= scale_height
        self.current_camera_info.P = np.array(self.original_camera_info.P) * scale_matrix

    @staticmethod
    def load_camera_info(filename):
        """Loads the camera calibration files.

        Loads the intrinsic camera calibration.

        Args:
            filename (:obj:`str`): filename of calibration files.

        Returns:
            :obj:`CameraInfo`: a CameraInfo message object

        """
        with open(filename, 'r') as stream:
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
    frame_thread = Thread(target=camera_node.start_capturing)
    frame_thread.start()
    # Keep it spinning to keep the node alive
    rospy.spin()
