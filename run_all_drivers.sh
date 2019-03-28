#!/bin/bash

set -e -x

source /home/duckiebot-interface/docker_setup.sh

source /home/duckiebot-interface/catkin_ws/devel/setup.bash

roslaunch duckietown all_drivers.launch veh:=$VEHICLE_NAME
