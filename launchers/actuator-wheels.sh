#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

ACTUATOR_NAME="base"
CONFIG_FILE="${ACTUATOR_NAME}/default"

exec python3 \
  -m wheels_driver_node.main \
    --actuator-name ${ACTUATOR_NAME} \
    --config ${CONFIG_FILE}

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
