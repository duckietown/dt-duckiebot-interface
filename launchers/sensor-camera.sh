#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

SENSOR_NAME="front_center"
CONFIG_FILE="${ROBOT_TYPE}/${ROBOT_HARDWARE}/${SENSOR_NAME}/default"

# The Jetson Orin Nano uses the same camera driver as the Jetson Nano.
if [ "$ROBOT_HARDWARE" = "jetson_orin_nano" ]; then
  ROBOT_HARDWARE="jetson_nano"
fi

exec python3 \
  -m camera_driver_node.${ROBOT_HARDWARE} \
    --sensor-name ${SENSOR_NAME} \
    --config ${CONFIG_FILE}

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
