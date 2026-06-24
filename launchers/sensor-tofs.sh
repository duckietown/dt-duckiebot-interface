#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

if [ "${ROBOT_TYPE}" == "duckiebot" ]; then
    SENSOR_NAME="front_center" dt-launcher-sensor-tof
elif [ "${ROBOT_TYPE}" == "duckiedrone" ]; then
    SENSOR_NAME="bottom" dt-launcher-sensor-tof &
    SENSOR_NAME="front" dt-launcher-sensor-tof &
    SENSOR_NAME="left" dt-launcher-sensor-tof &
    SENSOR_NAME="right" dt-launcher-sensor-tof &
    SENSOR_NAME="top" dt-launcher-sensor-tof &
fi

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
