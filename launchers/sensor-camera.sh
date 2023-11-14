#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)


# TODO: use different node for duckiedrone

# launching app
dt-exec roslaunch --wait \
    camera_driver camera_node.launch \
    veh:="${VEHICLE_NAME}" \
    param_file_name:="${ROBOT_TYPE}"


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
