#!/usr/bin/env bash

set -e

# install dependencies for raspicam_node
apt-get update
apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-camera-info-manager \
    libraspberrypi0 \
    libraspberrypi-dev \
    libpigpiod-if-dev
rm -rf /var/lib/apt/lists/*

set +e