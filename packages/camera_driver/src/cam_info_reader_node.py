#!/usr/bin/env python
import os.path
from camera_driver.camera_info import load_camera_info_2
import rospy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class CamInfoReaderNode(object):
    """Node containing the camera calibration file.

        The node publishes a message containing the camera intrinsic
        calibration. At startup looks for robot specific calibration, if
        not found uses a default file.

        Subscribers:
            ~compressed_image (ImageCompressed/Image): Description...
        Publishers:
            ~camera_info (CameraInfo): Description
    """

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing..." % (self.node_name))

        # Load parameters
        self.cali_file_name = self.setupParam("~cali_file_name", "default")
        self.image_type = self.setupParam("~image_type", "compressed")

        # Get path to calibration yaml file
        intrinsics_folder = "/data/config/calibrations/camera_intrinsic/"
        self.cali_file = (intrinsics_folder + self.cali_file_name + ".yaml")
        self.camera_info_msg = None

        # Locate calibration yaml file
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("[%s] Can't find calibration file: %s.\n Using default calibration instead."
                          % (self.node_name, self.cali_file))
            self.cali_file = (intrinsics_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Load calibration file
        self.camera_info_msg = load_camera_info_2(self.cali_file)

        # Print out and prepare message
        rospy.loginfo("[%s] Using calibration file: %s" % (self.node_name,
                                                           self.cali_file))
        self.camera_info_msg.header.frame_id = \
            rospy.get_namespace() + "camera_optical_frame"

        rospy.loginfo("[%s] CameraInfo: %s" % (self.node_name,
                                               self.camera_info_msg))

        img_type = CompressedImage if self.image_type == "compressed" else Image
        typemsg = "CompressedImage" if self.image_type == "compressed" else "Image"
        rospy.logwarn("[%s] ==============%s", self.node_name, typemsg)

        # Subscribers
        self.sub_img_compressed = rospy.Subscriber("~compressed_image",
                                                   img_type,
                                                   self.cbCompressedImage,
                                                   queue_size=1)
        # Publishers
        self.pub_camera_info = rospy.Publisher("~camera_info",
                                               CameraInfo,
                                               queue_size=1)

        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbCompressedImage(self, msg):
        """Callback of classamera info.

            Publishes the camera info whenever a compressed image (or image)
            gets published.

            Args:
                msg (img_type): image message
        """
        if self.camera_info_msg is not None:
            self.camera_info_msg.header.stamp = msg.header.stamp
            self.pub_camera_info.publish(self.camera_info_msg)

    def setupParam(self, param_name, default_value):
        """Parameter server handler.

            Gets the requested parameter, changes the value according to config
            file and prints it.

            Args:
                param_name (String): name of the parameter
                default_value(String): value of the paramter
        """
        value = rospy.get_param(param_name, default_value)

        # Write to parameter server for transparancy
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def on_shutdown(self):
        """Shutdown procedure."""
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('cam_info_reader', anonymous=False)
    # Create the TrafficLightNode object
    node = CamInfoReaderNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
