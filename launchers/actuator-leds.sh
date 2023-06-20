#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)


# prepare temporary launchfile
cat <<EOT > /tmp/actuator-leds.launch
<?xml version="1.0"?>
<launch>

    <include file="\$(find led_driver)/launch/led_driver_node.launch">
        <arg name="veh" value="${VEHICLE_NAME}"/>
        <arg name="robot_type" value="${VEHICLE_TYPE}"/>
    </include>

</launch>
EOT


LAUNCH_CONTENT=$(cat /tmp/actuator-leds.launch)
echo -e "LAUNCH FILE:\n\n${LAUNCH_CONTENT}\n"


# launching nodes
dt-exec roslaunch --wait \
    /tmp/actuator-leds.launch


# set LED color and intensity
INTENSITY=0.2
OFF="{r: 0, g: 0, b: 0, a: 0}"
WHITE="{r: 1, g: 1, b: 1, a: ${INTENSITY}}"
RED="{r: 1, g: 0, b: 0, a: ${INTENSITY}}"
GREEN="{r: 0, g: 1, b: 0, a: ${INTENSITY}}"
BLUE="{r: 0, g: 0, b: 1, a: ${INTENSITY}}"

read -r -d '' PATTERN << EOM
{rgb_vals: [
    ${WHITE},
    ${BLUE},
    ${OFF},
    ${GREEN},
    ${RED},
]}
EOM

rostopic pub --latch \
    /${VEHICLE_NAME}/led_driver_node/led_pattern \
    duckietown_msgs/LEDPattern \
    "${PATTERN}"


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
