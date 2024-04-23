#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

ACTUATOR_NAME="base"
CONFIG_FILE="${ACTUATOR_NAME}/default"

exec python3 \
  -m wheels_driver_node.main \
    --actuator-name ${ACTUATOR_NAME} \
    --config ${CONFIG_FILE}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
