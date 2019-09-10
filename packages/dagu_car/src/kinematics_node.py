#!/usr/bin/env python

import rospy
import yaml
import time
import os.path

from duckietown import DTROS
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from std_srvs.srv import EmptyResponse, Empty

class KinematicsNode(DTROS):
    """
    The `KinematicsNode` maps car commands send from various nodes to wheel commands
    that the robot can execute.

    The `KinematicsNode` performs both the inverse and forward kinematics calculations. Before
    these were implemented in separate nodes, but due to their similarity and parameter sharing,
    they are now combined.

    `KinematicsNode` utilises the car geometry as well as a number of tunining and limiting
    parameters to calculate the wheel commands that the wheels should execute in order for the
    robot to perform the desired car commands (inverse kinematics). Then it uses these wheel
    commands in order to do an open-loop velocity estimation (the forward kinematics part).

    TODO: Add link/explanation/illustration of the car geometry and the math used

    All the configuration parameters of this node can be changed dynamically while the node
    is running via `rosparam set` commands.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~gain (:obj:`float`): scaling factor applied to the desired velocity, default is 1.0
        ~trim (:obj:`float`): trimming factor that is typically used to offset differences in the
            behaviour of the left and right motors, it is recommended to use a value that results in
            the robot moving in a straight line when forward command is given, default is 0.0
        ~baseline (:obj:`float`): the distance between the two wheels of the robot, default is 0.1
        ~radius (:obj:`float`): radius of the wheel, default is 0.0318
        ~k (:obj:`float`): motor constant, assumed equal for both motors, default is 27.0
        ~limit (:obj:`float`): limits the final commands sent to the motors, default is 1.0
        ~v_max (:obj:`float`): limits the input velocity, default is 1.0
        ~omega_max (:obj:`float`): limits the input steering angle, default is 8.0

    Subscriber:
        ~car_cmd (:obj:`Twist2DStamped`): The requested car command

    Publisher:
        ~wheels_cmd (:obj:`WheelsCmdStamped`): The corresponding resulting wheel commands
        ~velocity (:obj:`Twist2DStamped`): The open-loop estimation of the robot velocity

    Service:
        ~save_calibration:
            Saves the current set of kinematics parameters (the ones in the Configuration section)
                to `/data/config/calibrations/kinematics/HOSTNAME.yaml`.

    """
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(KinematicsNode, self).__init__(node_name=node_name)

        # Get the vehicle name
        self.veh_name = rospy.get_namespace().strip("/")

        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~gain'] = None
        self.parameters['~trim'] = None
        self.parameters['~baseline'] = None
        self.parameters['~radius'] = None
        self.parameters['~k'] = None
        self.parameters['~limit'] = None
        self.parameters['~v_max'] = None
        self.parameters['~omega_max'] = None

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()
        self.updateParameters()

        # Prepare the save calibration service
        self.srv_save = rospy.Service("~save_calibration", Empty, self.cbSrvSaveCalibration)

        # Setup the publishers and subscribers
        self.sub_car_cmd = self.subscriber("~car_cmd", Twist2DStamped, self.car_cmd_callback)
        self.pub_wheels_cmd = self.publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.pub_velocity = self.publisher("~velocity", Twist2DStamped, queue_size=1)

        details = "[gain: %s trim: %s baseline: %s radius: %s k: %s limit: %s omega_max: %s v_max: %s]" % \
                  (self.parameters['~gain'], self.parameters['~trim'], self.parameters['~baseline'],
                   self.parameters['~radius'], self.parameters['~k'], self.parameters['~limit'],
                   self.parameters['~omega_max'], self.parameters['~v_max'])

        self.log("Initialized with: %s" % details)

    def readParamFromFile(self):
        """
        Reads the saved parameters from `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts the ROS paramaters for the node
        with the new values.

        """
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not exist! Using the default values." % fname, type='warn')
        else:
            with open(fname, 'r') as in_file:
                try:
                    yaml_dict = yaml.load(in_file)
                except yaml.YAMLError as exc:
                    self.log("YAML syntax error. File: %s fname. Exc: %s" %(fname, exc), type='fatal')
                    rospy.signal_shutdown()
                    return

            # Set parameters using value in yaml file
            if yaml_dict is None:
                # Empty yaml file
                return
            for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
                param_value = yaml_dict.get(param_name)
                if param_name is not None:
                    rospy.set_param("~"+param_name, param_value)
                else:
                    # Skip if not defined, use default value instead.
                    pass

    def getFilePath(self, name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific calibration file

        """
        cali_file_folder = '/data/config/calibrations/kinematics/'
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file

    def cbSrvSaveCalibration(self, req):
        """
        Saves the current kinematics paramaters to a robot-specific file at
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            req: dummy variable to be callback-complacent. not used.

        Returns:
            EmptyResponse

        """

        # Write to a yaml file
        self.updateParameters()
        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "gain": self.parameters['~gain'],
            "trim": self.parameters['~trim'],
            "baseline": self.parameters['~baseline'],
            "radius": self.parameters['~radius'],
            "k": self.parameters['~k'],
            "limit": self.parameters['~limit'],
            "v_max": self.parameters['~v_max'],
            "omega_max": self.parameters['~omega_max'],
        }

        # Write to file
        file_name = self.getFilePath(self.veh_name)
        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False))

        # Printout
        saved_details = "[gain: %s trim: %s baseline: %s radius: %s k: %s limit: %s omega_max: %s v_max: %s]" % \
        (self.parameters['~gain'], self.parameters['~trim'], self.parameters['~baseline'], self.parameters['~radius'],
         self.parameters['~k'], self.parameters['~limit'], self.parameters['~omega_max'], self.parameters['~v_max'])
        self.log("Saved kinematic calibration to %s with values: %s" % (file_name, saved_details))

        return EmptyResponse()

    def car_cmd_callback(self, msg_car_cmd):
        """
        A callback that reposponds to received `car_cmd` messages by calculating the
        corresponding wheel commands, taking into account the robot geometry, gain and trim
        factors, and the set limits. These wheel commands are then published for the motors to use.
        The resulting linear and angular velocities are also calculated and published.

        Args:
            msg_car_cmd (:obj:`Twist2DStamped`): desired car command

        """

        # if the parameters have been updated, check them and warn the user if there's something wrong with them
        if self.parametersChanged:
            if self.parameters['~limit'] < 0:
                self.log("The new limit %s is smaller than min of 0" % self.parameters['~limit'], type='warn')
            elif self.parameters['~limit'] > 1:
                self.log("The new limit %s is larger than max of 1" % self.parameters['~limit'], type='warn')

            for param in ['gain', 'baseline', 'radius', 'k', 'v_max', 'omega_max']:
                if self.parameters['~'+param] < 0:
                    self.log("The new value of %s is negative, should be positive." % param, type='warn')
            self.parametersChanged = False

        # INVERSE KINEMATICS PART

        # trim the desired commands such that they are within the limits:
        msg_car_cmd.v = self.trim(msg_car_cmd.v,
                                  low=-self.parameters['~v_max'], high=self.parameters['~v_max'])
        msg_car_cmd.omega = self.trim(msg_car_cmd.omega,
                                      low=-self.parameters['~omega_max'], high=self.parameters['~omega_max'])

        # assuming same motor constants k for both motors
        k_r = self.parameters['~k']
        k_l = self.parameters['~k']

        # adjusting k by gain and trim
        k_r_inv = (self.parameters['~gain'] + self.parameters['~trim']) / k_r
        k_l_inv = (self.parameters['~gain'] - self.parameters['~trim']) / k_l

        omega_r = (msg_car_cmd.v + 0.5 * msg_car_cmd.omega * self.parameters['~baseline']) / self.parameters['~radius']
        omega_l = (msg_car_cmd.v - 0.5 * msg_car_cmd.omega * self.parameters['~baseline']) / self.parameters['~radius']

        # conversion from motor rotation rate to duty cycle
        # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r, -self.parameters['~limit'], self.parameters['~limit'])
        u_l_limited = self.trim(u_l, -self.parameters['~limit'], self.parameters['~limit'])

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = msg_car_cmd.header.stamp
        msg_wheels_cmd.vel_right = u_r_limited
        msg_wheels_cmd.vel_left = u_l_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

        # FORWARD KINEMATICS PART

        # Conversion from motor duty to motor rotation rate
        omega_r = msg_wheels_cmd.vel_right / k_r_inv
        omega_l = msg_wheels_cmd.vel_left / k_l_inv

        # Compute linear and angular velocity of the platform
        v = (self.parameters['~radius'] * omega_r + self.parameters['~radius'] * omega_l) / 2.0
        omega = (self.parameters['~radius'] * omega_r - self.parameters['~radius'] * omega_l) / \
                self.parameters['~baseline']

        # Put the v and omega into a velocity message and publish
        msg_velocity = Twist2DStamped()
        msg_velocity.header = msg_wheels_cmd.header
        msg_velocity.v = v
        msg_velocity.omega = omega
        self.pub_velocity.publish(msg_velocity)

    def trim(self, value, low, high):
        """
        Trims a value to be between some bounds.

        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound

        Returns:
            the trimmed value
        """

        return max(min(value, high), low)

if __name__ == '__main__':
    # Initialize the node
    kinematics_node = KinematicsNode(node_name='kinematics_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
