#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

roslaunch --wait \
  duckiebot_interface all_drivers.launch \
    veh:="$VEHICLE_NAME" \
    robot_type:="$ROBOT_TYPE" \
    robot_configuration:="$ROBOT_CONFIGURATION"
