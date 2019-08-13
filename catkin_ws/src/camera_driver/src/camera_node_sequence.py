#!/usr/bin/env python
import io
import thread
import os
import yaml

from duckietown_msgs.msg import BoolStamped
#from duckietown_utils import get_duckiefleet_root
from picamera import PiCamera
from picamera.array import PiRGBArray
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from camera_driver.camera_info import load_camera_info_2


class CameraNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        # from cam_info_reader_node
        self.config = self.setupParam("~config", "baseline")
        # TODO cali_file_name is not needed and should be the robot name by default
        self.cali_file_name = self.setupParam("~cali_file_name", "default")
        self.image_type = self.setupParam("~image_type", "compressed")
        self.pub_camera_info = rospy.Publisher(
            "~camera_info", CameraInfo, queue_size=1)

        # from cam_info_reader_node end

        self.framerate_high = self.setupParam("~framerate_high", 30.0)
        self.framerate_low = self.setupParam("~framerate_low", 15.0)
        self.res_w = self.setupParam("~res_w", 640)
        self.res_h = self.setupParam("~res_h", 480)

        self.image_msg = CompressedImage()

        # Setup PiCamera

        self.camera = PiCamera()
        self.framerate = self.framerate_high  # default to high
        self.camera.framerate = self.framerate
        self.camera.resolution = (self.res_w, self.res_h)

        # For intrinsic calibration
        if not 'DUCKIEFLEET_ROOT' in os.environ:
            msg = 'DUCKIEFLEET_ROOT not defined - setting calibration dir to default /data/config'
            duckiefleet_root = '/data/config'
        else:
            duckiefleet_root = os.environ['DUCKIEFLEET_ROOT']

        # from cam_info_reader_node
        # Get path to calibration yaml file
        self.cali_file = (duckiefleet_root + "/calibrations/camera_intrinsic/"
                          + self.cali_file_name + ".yaml")
        self.camera_info_msg = None
        # Load calibration yaml file
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("[%s] Can't find calibration file: %s.\nUsing default calibration instead."
                          % (self.node_name, self.cali_file))
            self.cali_file = (duckiefleet_root +
                              "/calibrations/camera_intrinsic/default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Print out and prepare message
        rospy.loginfo("[%s] Using calibration file: %s" %
                      (self.node_name, self.cali_file))
        self.camera_info_msg = load_camera_info_2(self.cali_file)
        self.camera_info_msg.header.frame_id = rospy.get_namespace() + \
            "camera_optical_frame"
        rospy.loginfo("[%s] CameraInfo: %s" %
                      (self.node_name, self.camera_info_msg))
        # self.timer_pub = rospy.Timer(rospy.Duration.from_sec(1.0/self.pub_freq),self.cbTimer)

        img_type = CompressedImage if self.image_type == "compressed" else Image
        typemsg = "CompressedImage" if self.image_type == "compressed" else "Image"
        rospy.logwarn("[%s] ==============%s", self.node_name, typemsg)
        # from cam_info_reader_node end

        self.cali_file_folder = duckiefleet_root + "/calibrations/camera_intrinsic/"

        self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"

        self.has_published = False
        self.pub_img = rospy.Publisher(
            "~image/compressed", CompressedImage, queue_size=1)
        self.sub_switch_high = rospy.Subscriber(
            "~framerate_high_switch", BoolStamped, self.cbSwitchHigh, queue_size=1)

        # Create service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service(
            "~set_camera_info", SetCameraInfo, self.cbSrvSetCameraInfo)

        self.stream = io.BytesIO()

        #self.camera.exposure_mode = 'off'
        # self.camera.awb_mode = 'off'

        self.is_shutdown = False
        self.update_framerate = False
        # Setup timer
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbSwitchHigh(self, switch_msg):
        print switch_msg
        if switch_msg.data and self.framerate != self.framerate_high:
            self.framerate = self.framerate_high
            self.update_framerate = True
        elif not switch_msg.data and self.framerate != self.framerate_low:
            self.framerate = self.framerate_low
            self.update_framerate = True

    def startCapturing(self):
        rospy.loginfo("[%s] Start capturing." % (self.node_name))
        while not self.is_shutdown and not rospy.is_shutdown():
            gen = self.grabAndPublish(
                self.stream, self.pub_img, self.pub_camera_info)
            try:
                self.camera.capture_sequence(
                    gen, 'jpeg', use_video_port=True, splitter_port=0)
            except StopIteration:
                pass
            # print "updating framerate"
            self.camera.framerate = self.framerate
            self.update_framerate = False

        self.camera.close()
        rospy.loginfo("[%s] Capture Ended." % (self.node_name))

    def grabAndPublish(self, stream, publisher, cam_info_publisher):
        while not self.update_framerate and not self.is_shutdown and not rospy.is_shutdown():
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

            self.camera_info_msg.header.stamp = stamp
            cam_info_publisher.publish(self.camera_info_msg)
            # Clear stream
            stream.seek(0)
            stream.truncate()

            if not self.has_published:
                rospy.loginfo("[%s] Published the first image." %
                              (self.node_name))
                self.has_published = True

            rospy.sleep(rospy.Duration.from_sec(0.001))

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparancy
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def onShutdown(self):
        rospy.loginfo("[%s] Closing camera." % (self.node_name))
        self.is_shutdown = True
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

    def cbSrvSetCameraInfo(self, req):
        # TODO: save req.camera_info to yaml file
        rospy.loginfo("[cbSrvSetCameraInfo] Callback!")
        filename = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.saveCameraInfo(req.camera_info, filename)
        response.status_message = "Write to %s" % filename  # TODO file name
        return response

    def saveCameraInfo(self, camera_info_msg, filename):
        # Convert camera_info_msg and save to a yaml file
        rospy.loginfo("[saveCameraInfo] filename: %s" % (filename))

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
                 'image_height': camera_info_msg.height,
                 'camera_name': rospy.get_name().strip("/"),  # TODO check this
                 'distortion_model': camera_info_msg.distortion_model,
                 'distortion_coefficients': {'data': camera_info_msg.D, 'rows': 1, 'cols': 5},
                 'camera_matrix': {'data': camera_info_msg.K, 'rows': 3, 'cols': 3},
                 'rectification_matrix': {'data': camera_info_msg.R, 'rows': 3, 'cols': 3},
                 'projection_matrix': {'data': camera_info_msg.P, 'rows': 3, 'cols': 4}}

        rospy.loginfo("[saveCameraInfo] calib %s" % (calib))

        try:
            f = open(filename, 'w')
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False


if __name__ == '__main__':
    rospy.init_node('camera', anonymous=False)
    camera_node = CameraNode()
    rospy.on_shutdown(camera_node.onShutdown)
    thread.start_new_thread(camera_node.startCapturing, ())
    rospy.spin()
