#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# If the robot type is duckiedrone and ROBOT_HARDWARE is virtual, then we need to launch the betaflight-SITL
if [ "${ROBOT_TYPE}" == "duckiedrone" ] && [ "${ROBOT_HARDWARE}" == "virtual" ]; then
    dt-exec /usr/bin/betaflight/launch_betaflight.sh &
fi
# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
