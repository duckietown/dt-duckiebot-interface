#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

SENSOR_NAME="interaction_plate"
CONFIG_FILE="${SENSOR_NAME}/default"

exec python3 \
  -m button_driver_node.main \
    --sensor-name ${SENSOR_NAME} \
    --config ${CONFIG_FILE}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
