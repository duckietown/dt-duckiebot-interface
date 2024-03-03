#!/bin/bash


# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
exec python3 -m duckiebot_interface.launch.main \
    --camera \
    --tof \
    --wheels \
    --wheel-encoders \
    --leds \
    --display \
    --button \
    --imu \
    --display-renderer


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
