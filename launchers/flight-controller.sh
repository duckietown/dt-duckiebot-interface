#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

CONFIG_FILE="${ROBOT_HARDWARE}/default"

# Set the owner of /data/config/nodes to the current user.
dt-launcher-betaflight-sitl &
sleep 1
sudo mkdir -p /data/config/nodes
sudo chown -R ${USER}:${USER} /data/config/nodes
exec python3 \
  -m flight_controller_node.main \
    --config ${CONFIG_FILE}

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
