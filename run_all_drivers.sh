#!/bin/bash

set -e -x

source $SOURCE_DIR/docker_setup.sh

source $SOURCE_DIR/catkin_ws/devel/setup.bash

roslaunch duckietown all_drivers.launch veh:=$VEHICLE_NAME
