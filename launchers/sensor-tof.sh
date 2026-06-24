#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

CONFIG_FILE="default"

exec python3 \
  -m tof_driver_node.main \
    --sensor-name ${SENSOR_NAME} \
    --config ${ROBOT_TYPE}/${SENSOR_NAME}/${CONFIG_FILE}

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
