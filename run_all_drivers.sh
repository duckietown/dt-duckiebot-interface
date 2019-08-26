#!/bin/bash

set -e -x

roslaunch duckiebot_interface all_drivers.launch veh:=$VEHICLE_NAME
