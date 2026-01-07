#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

PROXY_NAME="simulator"
CONFIG_FILE="default"

if [ "${ROBOT_HARDWARE}" == "virtual" ]; then
  exec python3 \
  -m mavlink_proxy_node.main \
    --proxy-name ${PROXY_NAME} \
    --config ${CONFIG_FILE}
fi

echo "mavlink-proxy not needed for real duckiedrones"
exec sleep infinity

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
