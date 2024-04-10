#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

CONFIG_FILE="default"
CONFIG_FILE="${ROBOT_HARDWARE}/default"

# Set owner of /data/config/nodes to the current user
dt-launcher-betaflight-sitl&
sleep 1
sudo mkdir -p /data/config/nodes
sudo chown -R ${USER}:${USER} /data/config/nodes
exec python3 \
  -m flight_controller_node.main \
    --config ${CONFIG_FILE}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
