#!/bin/bash

set -e -x

source $CATKIN_WS_DIR/src/duckiebot-interface/docker_setup.sh

source $CATKIN_WS_DIR/devel/setup.bash

roslaunch duckietown all_drivers.launch veh:=$VEHICLE_NAME
