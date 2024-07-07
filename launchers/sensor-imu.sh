#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

SENSOR_NAME="base"
CONFIG_FILE="${ROBOT_TYPE}/${SENSOR_NAME}/default"

if [ "${ROBOT_HARDWARE}" == "virtual" ]; then
  echo "Sensor 'imu' not implemented for Virtual robots"
  exec sleep infinity
fi

exec python3 \
  -m imu_driver_node.main \
    --sensor-name ${SENSOR_NAME} \
    --config ${CONFIG_FILE}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
