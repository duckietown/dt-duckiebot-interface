#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

CONFIG_FILE="default"

exec python3 \
  -m wheel_encoder_driver_node.main \
    --side ${SIDE} \
    --config ${SIDE}/${CONFIG_FILE}

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
