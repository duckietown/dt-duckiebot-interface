#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

PROXY_NAME="simulator"
CONFIG_FILE="default"

if [ "${ROBOT_HARDWARE}" == "virtual" ]; then
  exec python3 \
  -m mavlink_proxy_node.main \
    --proxy-name ${PROXY_NAME} \
    --config ${CONFIG_FILE}
fi

echo "mavlink-proxy not needed for physical Duckiedrones."
exec sleep infinity

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
