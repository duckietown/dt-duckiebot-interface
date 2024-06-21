#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

SENSOR_NAME="front_center"
CONFIG_FILE="default"

exec python3 \
  -m tof_driver_node.main \
    --sensor-name ${SENSOR_NAME} \
    --config ${ROBOT_TYPE}/${SENSOR_NAME}/${CONFIG_FILE}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
