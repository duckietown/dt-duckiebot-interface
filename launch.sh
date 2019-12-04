#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
dt_exec roslaunch --wait duckiebot_interface all_drivers.launch veh:=$VEHICLE_NAME robot_type:=$ROBOT_TYPE
