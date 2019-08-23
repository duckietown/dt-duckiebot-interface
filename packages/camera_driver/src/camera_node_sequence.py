#!/usr/bin/env python
import io
import thread
import os
import yaml
import rospy

from dtros import DTROS
from picamera import PiCamera
from sensor_msgs.msg import CompressedImage
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

class CameraNode(DTROS):
    """Handles the imagery.

    The node handles the image stream, initializing it, publishing frames
    according to the required frequency and stops it at shutdown.
    `Picamera <https://picamera.readthedocs.io/>`_ is used for handling the image stream.

    Note that only one :obj:`PiCamera` object should be used at a time. If another node tries to start
    an instance while this node is running, it will likely fail with an `Out of resource` exception.

    The node can publish both at high and low framerate settings which are configurable parameters. Publishing to
    the `~framerate_high_switch` topic can switch between the two settings. The high setting is used by default.

    Configuration:
        ~framerate (float): The camera image acquisition framerate, default is 30.0 fps
        ~res_w (int): The desired width of the acquired image, default is 640px
        ~res_h (int): The desired height of the acquired image, default is 480px
        ~exposure_mode (string): PiCamera exposure mode, one of `these <https://picamera.readthedocs.io/en/latest/api_camera.html?highlight=sport#picamera.PiCamera.exposure_mode>`_, default is `sports`

    Publisher:
        ~image/compressed (CompressedImage): The acquired camera images

    Service:
        ~set_camera_info:
            Saves a provided camera info to `/data/config/calibrations/camera_intrinsic/NAMESPACE.yaml`.

            input:
                camera_info (sensor_msgs/CameraInfo) The camera information to save

            outputs:
                success (bool): `True` if the call succeeded
                status_message (string): Used to give details about success

    """

    def __init__(self):

        # Initialize the DTROS parent class
        super(CameraNode, self).__init__()

        # Add the node parameters to the parameters dictionary
        self.parameters['~framerate'] = None
        self.parameters['~res_w'] = None
        self.parameters['~res_h'] = None
        self.parameters['~exposure_mode'] = None
        self.updateParameters(None)

        self.image_msg = CompressedImage()

        # Setup PiCamera
        self.camera = PiCamera()
        self.camera.framerate = self.parameters['~framerate']
        self.camera.resolution = (self.parameters['~res_w'], self.parameters['~res_h'])
        self.camera.exposure_mode = self.parameters['~exposure_mode']

        # For intrinsic calibration
        self.cali_file_folder = '/data/config/calibrations/camera_intrinsic/'

        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'

        self.has_published = False
        self.pub_img = rospy.Publisher("~image/compressed",
                                       CompressedImage,
                                       queue_size=1)

        # Create service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service("~set_camera_info",
                                                 SetCameraInfo,
                                                 self.cbSrvSetCameraInfo)
        self.stream = io.BytesIO()

        self.is_shutdown = False

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
            gen = self.grabAndPublish(self.stream, self.pub_img)
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
            self.parametersChanged = False
            self.log("Parameters updated.")

        self.camera.close()
        self.log("Capture Ended.")

    def grabAndPublish(self, stream, publisher):
        """Captures a frame from stream and publishes it.

            If the stream is stable (no parameter updates or shutdowns),
            grabs a frame, creates the image message and publishes it. If there is a paramter change,
            it does raises StopIteration exception which is caught by `startCapturing`. It updates the
            camera parameters and restarts the recording.

            Args:
                stream (BytesIO): imagery stream
                publisher (Publisher): publisher of topic
        """
        while not (self.parametersChanged or self.is_shutdown or rospy.is_shutdown()):
            yield stream
            # Construct image_msg
            # Grab image from stream
            stamp = rospy.Time.now()
            stream.seek(0)
            stream_data = stream.getvalue()
            # Generate compressed image
            image_msg = CompressedImage()
            image_msg.format = "jpeg"
            image_msg.data = stream_data

            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            publisher.publish(image_msg)

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
                camera_info_msg (CameraInfo): Camera Info containg calibration
                filename (String): filename where to save calibration
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


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('camera', anonymous=False)
    # Create the TrafficLightNode object
    camera_node = CameraNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(camera_node.onShutdown)
    thread.start_new_thread(camera_node.startCapturing, ())
    # Keep it spinning to keep the node alive
    rospy.spin()
