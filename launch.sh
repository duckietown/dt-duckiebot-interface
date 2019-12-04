#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
# read robot type (TODO: update PATH)
ROBOT_TYPE=`cat /data/stats/init_sd_card/parameters/robot_type`

# launch duckiebot_interface
dt_exec roslaunch --wait duckiebot_interface all_drivers.launch veh:=$VEHICLE_NAME robot_type:=$ROBOT_TYPE
