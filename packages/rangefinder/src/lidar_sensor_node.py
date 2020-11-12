#!/usr/bin/env python3

import sys
import os
import rospy
import signal
import time
from std_msgs.msg import Empty
from sensor_msgs.msg import Range
import numpy as np
import RPi.GPIO as GPIO
import VL53L1X


#Each of the four spinning lidar sensor nodes
#gets passed the specific i2c channel it is meant to
#use through the local param in the launch file as 
#a hex string '0x3A' for example, must convert to 
#an int





class Lidar(object):
    """A class that reads, analyzes, and publishes IR sensor data.

    Publisher:
    /pidrone/lidar_sensor
    /pidrone/heartbeat/infrared  #TODO rename all IR instances to either
    lidar or rangefinder on case by case basis
    """
    ### this block is to set up the lidar range finders
    # sys.path.insert(0, "build/lib.linux-armv7l-2.7/")
    ### the try/except block is to allow compatibility
    ### with the drones that use IR sensors
    

    def __init__(self, i2c, max_range):
        self.max_range = max_range
        self.i2c = i2c
        ### development smoother
        GPIO.setmode(GPIO.BCM)
        mode = GPIO.getmode()
        #then innit the first one
        self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=i2c)
        self.tof.open()
        self.tof.start_ranging(1)
        #this is to increase frequency
        budget = 1
        inter = 1
        self.tof.set_timing(budget, inter)
        ### end of setup block

        ### set maxrange param
        try:
            rospy.get_param("maxrange")
        except KeyError:
            #not set yet
            rospy.set_param("maxrange", str(max_range))

    def get_range(self):
        """need to convert from mm to meters"""
        self.distance = self.tof.get_distance() / 1000.0


    def publish_range(self, input_range):
        """Create and publish the Range message to publisher."""
        msg = Range()
        msg.max_range = self.max_range #different max for lidar version
        msg.min_range = 0.0   #range in meters
        msg.range = input_range
        msg.header.frame_id = "base"
        msg.header.stamp = rospy.Time.now()
        self.range_pub.publish(msg)

    def ctrl_c_handler(self, signal, frame):
        """Gracefully quit the infrared_pub node"""
        print("\nCaught ctrl-c! Stopping node.")
        sys.exit()

def main():
    """Start the ROS node, create the publishers, and continuosly update and
    publish the lidar sensor data"""
    time.sleep(1) # allow 1 second for the i2c remapping to occur
    # ROS Setup
    ###########
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    i2c_id= rospy.get_param("~i2c_channel_id")
    i2c_base = "0x3"
    i2c = str(i2c_base)+str(i2c_id)
    i2c= int(i2c, 16)
    max_range= "3.0"
    max_range= float(max_range)

    

    #convert i2c channel from hex string to int

    # create IR object
    li = Lidar(i2c, max_range)

    # Publishers
    ############
    li.range_pub = rospy.Publisher('lidar_sensor_node_' + str(i2c_id+1), Range, queue_size=1)
    li.heartbeat_pub = rospy.Publisher('heartbeat/lidar_sensor_node_'+ str(i2c_id+1), Empty, queue_size=1)
    print('Starting Lidar sensor '+str(i2c_id+1))

    # Non-ROS Setup
    ###############
    # set the while loop frequency
    r = rospy.Rate(100)
    # set up the ctrl-c handler
    signal.signal(signal.SIGINT, li.ctrl_c_handler)

    while not rospy.is_shutdown():
        li.heartbeat_pub.publish(Empty())
        li.get_range()
        li.publish_range(li.distance)
        r.sleep()



if __name__ == "__main__":
    main()
