#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# this is necessary for the camera pipeline to work on the Jetson Nano
if [ "${ROBOT_HARDWARE}" == "jetson_nano" ]; then
    export LD_PRELOAD=${LD_PRELOAD}:/usr/lib/aarch64-linux-gnu/libGLdispatch.so
fi

# If the robot type is duckiedrone and ROBOT_HARDWARE is virtual, then we need to launch the betaflight-SITL
if [ "${ROBOT_TYPE}" == "duckiedrone" ] && [ "${ROBOT_HARDWARE}" == "virtual" ]; then
    dt-exec /usr/bin/betaflight/launch_betaflight.sh
fi

# set module's health
dt-set-module-healthy

# launch robot-type specific launcher
dt-exec dt-launcher-default-${ROBOT_TYPE}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
