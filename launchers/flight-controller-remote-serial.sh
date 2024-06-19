#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

CONFIG_FILE="default"
CONFIG_FILE="${ROBOT_HARDWARE}/default"

# Set owner of /data/config/nodes to the current user
dt-launcher-betaflight-sitl&
exec python3 packages/flight_controller_driver/utils/network_serial.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
