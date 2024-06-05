#!/usr/bin/env bash

# this is necessary for the camera pipeline to work on the Jetson Nano
if [ "${ROBOT_HARDWARE}" == "jetson_nano" ]; then
    export LD_PRELOAD=${LD_PRELOAD}:/usr/lib/aarch64-linux-gnu/libGLdispatch.so
fi
