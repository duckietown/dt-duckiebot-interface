#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

ACTUATOR_NAME="interaction_plate"
CONFIG_FILE="${ACTUATOR_NAME}/default"

if [ "${ROBOT_HARDWARE}" == "virtual" ]; then
  echo "Actuator 'display' not implemented for virtual robots."
  exec sleep infinity
fi

dt-exec python3 \
  -m display_driver_node.main \
    --actuator-name ${ACTUATOR_NAME} \
    --config ${CONFIG_FILE}

dt-exec python3 \
  -m display_renderer_node.main \
    --config ${CONFIG_FILE}

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
