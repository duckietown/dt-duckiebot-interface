#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

NETWORK_RENDERER="
<!-- networking: renders connectivity status -->
<include file=\"\$(find display_renderers)/launch/networking_renderer_node.launch\">
    <arg name=\"veh\" value=\"${VEHICLE_NAME}\"/>
</include>
"

HEALTH_RENDERER="
<!-- health: renders health and usage info about the robot -->
<include file=\"\$(find display_renderers)/launch/health_renderer_node.launch\">
    <arg name=\"veh\" value=\"${VEHICLE_NAME}\"/>
</include>
"


# prepare temporary launchfile
cat <<EOT > /tmp/actuator-lcd-display.launch
<?xml version="1.0"?>
<launch>

    <!-- drivers: renders fragments on the display -->
    <remap from="display_driver_node/button" to="button_driver_node/event"/>
    <include file="\$(find display_driver)/launch/display_driver_node.launch">
        <arg name="veh" value="${VEHICLE_NAME}"/>
    </include>

    ${HEALTH_RENDERER}
    ${NETWORK_RENDERER}

</launch>
EOT


LAUNCH_CONTENT=$(cat /tmp/actuator-lcd-display.launch)
echo -e "LAUNCH FILE:\n\n${LAUNCH_CONTENT}\n"


# launching nodes
dt-exec roslaunch --wait \
    /tmp/actuator-lcd-display.launch


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
