#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

SENSOR_NAME="front-center"
CONFIG_FILE="${ROBOT_TYPE}/${ROBOT_HARDWARE}/default"

exec python3 \
  -m camera_driver_node.${ROBOT_HARDWARE} \
    --sensor-name ${SENSOR_NAME} \
    --config ${CONFIG_FILE}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
