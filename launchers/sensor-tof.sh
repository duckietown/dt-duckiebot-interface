#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

CONFIG_FILE="default"

if [ "${ROBOT_TYPE}" == "duckiebot" ]; then
    # duckiebots have a ToF sensor on the front-center
    SENSOR_NAME="front_center"
elif [ "${ROBOT_TYPE}" == "duckiedrone" ]; then
    # duckiedrone have a ToF sensor on the bottom
    SENSOR_NAME="bottom"
else
    echo "This robot does not have a ToF sensor."
    exit 1
fi


exec python3 \
  -m tof_driver_node.main \
    --sensor-name ${SENSOR_NAME} \
    --config ${ROBOT_TYPE}/${SENSOR_NAME}/${CONFIG_FILE}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
