#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

SENSOR_NAME="front_center"
CONFIG_FILE="${ROBOT_TYPE}/${ROBOT_HARDWARE}/${SENSOR_NAME}/default"

# The jetson orin nano uses the same camera driver as the jetson nano 
if [ "$ROBOT_HARDWARE" = "jetson_orin_nano" ]; then
  ROBOT_HARDWARE="jetson_nano"
fi

exec python3 \
  -m camera_driver_node.${ROBOT_HARDWARE} \
    --sensor-name ${SENSOR_NAME} \
    --config ${CONFIG_FILE}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
