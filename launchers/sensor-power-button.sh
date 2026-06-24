#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

SENSOR_NAME="interaction_plate"
CONFIG_FILE="${SENSOR_NAME}/default"

if [ "${ROBOT_HARDWARE}" == "virtual" ]; then
  echo "Sensor 'power-button' not implemented for virtual robots."
  exec sleep infinity
fi

exec python3 \
  -m button_driver_node.main \
    --sensor-name ${SENSOR_NAME} \
    --config ${CONFIG_FILE}

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
