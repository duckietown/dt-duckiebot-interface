#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

ACTUATOR_NAME="interaction_plate"
CONFIG_FILE="${ACTUATOR_NAME}/default"

if [ "${ROBOT_HARDWARE}" == "virtual" ]; then
  echo "Actuator 'display' not implemented for Virtual robots"
  exec sleep infinity
fi


# display driver
dt-exec python3 \
  -m display_driver_node.main \
    --actuator-name ${ACTUATOR_NAME} \
    --config ${CONFIG_FILE}

# default renderers
dt-exec python3 \
  -m display_renderer_node.main \
    --config ${CONFIG_FILE}

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
