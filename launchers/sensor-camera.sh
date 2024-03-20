#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# this is necessary for the camera pipeline to work on the Jetson Nano
#if [ "${ROBOT_HARDWARE}" == "jetson_nano" ]; then
#    export LD_PRELOAD=${LD_PRELOAD}:/usr/lib/aarch64-linux-gnu/libGLdispatch.so
#fi

CONFIG_FILE="${ROBOT_TYPE}/${ROBOT_HARDWARE}/default"

exec python3 \
  -m camera_driver_node.${ROBOT_HARDWARE} \
    --config ${CONFIG_FILE}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
