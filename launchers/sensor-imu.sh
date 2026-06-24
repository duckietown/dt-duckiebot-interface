#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

SENSOR_NAME="base"
CONFIG_FILE="${ROBOT_TYPE}/${SENSOR_NAME}/default"

exec python3 \
  -m imu_driver_node.main \
    --sensor-name ${SENSOR_NAME} \
    --config ${CONFIG_FILE}

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
