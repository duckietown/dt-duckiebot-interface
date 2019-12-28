#!/bin/bash

source /environment.sh

# initialize launch file
dt_launchfile_init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable CODE_DIR to know the absolute path to your code
# NOTE: Use `dt_exec COMMAND` to run the main process (blocking process)

# TODO: this should not run roscore, have a separate container for it and use --wait here instead

# launching app
dt_exec roslaunch duckiebot_interface all_drivers.launch veh:=$VEHICLE_NAME robot_type:=$ROBOT_TYPE


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# terminate launch file
dt_launchfile_terminate
