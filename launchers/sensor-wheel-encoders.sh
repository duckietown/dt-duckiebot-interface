#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

CONFIG_FILE="default"

# left wheel encoder
dt-exec python3 \
  -m wheel_encoder_driver_node.main \
    --side left \
    --config left/${CONFIG_FILE}

# right wheel encoder
dt-exec python3 \
  -m wheel_encoder_driver_node.main \
    --side right \
    --config right/${CONFIG_FILE}

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
