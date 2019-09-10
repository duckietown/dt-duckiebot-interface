#!/usr/bin/env python

import rospy

from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped, FSMState

class CarCmdSwitchNode(DTROS):
    """

    Warnings:
        This node should be seriously revisted such that it uses DTROS to deactivate nodes that are not
        being listened to and to actively register topics and states instead of using a config file for that!

        For the same reason, the documentation of this node is not up to par.

    Subscriptions "~mode": duckietown_msgs/FSMState. Current control mode of the duckiebot
    Other subscriptions specified by the parameters

    Publications "~cmd" duckietown_msgs/Twist2DStamped.
    """
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(CarCmdSwitchNode, self).__init__(node_name=node_name)

        # Read parameters
        self.mappings = rospy.get_param("~mappings")
        source_topic_dict = rospy.get_param("~source_topics")
        self.current_src_name = "joystick"

        # Construct publisher
        self.pub_cmd = self.publisher("~cmd",Twist2DStamped,queue_size=1)

        # Construct subscribers
        self.sub_fsm_state = self.subscriber(rospy.get_param("~mode_topic"),FSMState,self.cbFSMState)

        self.sub_dict = dict()
        for src_name, topic_name in source_topic_dict.items():
            self.sub_dict[src_name] = self.subscriber(topic_name,Twist2DStamped,self.cbWheelsCmd,callback_args=src_name)

        self.log("Initialized. ")
        self.log("Sources: %s"%str(source_topic_dict))

    def cbFSMState(self,fsm_state_msg):
        self.current_src_name = self.mappings.get(fsm_state_msg.state)
        if self.current_src_name == "stop":
            self.pubStop()
            self.log("Car cmd switched to STOP in state %s." %(fsm_state_msg.state))
        elif self.current_src_name is None:
            self.log("FSMState %s not handled. No msg pass through the switch." %(fsm_state_msg.state), type='warn')
        else:
            self.log("Car cmd switched to %s in state %s." %(self.current_src_name,fsm_state_msg.state))

    def cbWheelsCmd(self,msg,src_name):
        if src_name == self.current_src_name:
            self.pub_cmd.publish(msg)

    def pubStop(self):
        msg = Twist2DStamped()
        msg.v = 0
        msg.omega = 0
        self.pub_cmd.publish(msg)

if __name__ == '__main__':
    # Create the DaguCar object
    car_cmd_switch_node = CarCmdSwitchNode(node_name='car_cmd_switch_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
