#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

CONFIG_FILE="${ROBOT_HARDWARE}/default"

# Set the owner of /data/config/nodes to the current user.
dt-launcher-betaflight-sitl &
exec python3 "${DT_PROJECT_PATH}/packages/flight_controller_driver/utils/network_serial.py"

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
