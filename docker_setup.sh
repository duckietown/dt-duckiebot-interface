#!/usr/bin/env bash

MY_IP=$(hostname -I | cut -d " " -f 1)
export ROS_IP=${MY_IP}
echo "Setting ROS_IP to host IP, which is $ROS_IP"
if [ ! -z "$ROS_MASTER_URI" ]; then
    echo "ROS_MASTER_URI was externally set to \"$ROS_MASTER_URI\", skipping configuration."
elif [ ! -z "$DUCKIEBOT_NAME" ] && [ ! -z "$DUCKIEBOT_IP" ]; then # We are running on the Desktop
    duckiebot_binding="$DUCKIEBOT_IP $DUCKIEBOT_NAME $DUCKIEBOT_NAME.local"
    echo "Writing \"$duckiebot_binding\" into /etc/hosts"
    echo $duckiebot_binding >> /etc/hosts
    export ROS_MASTER_URI="http://$DUCKIEBOT_NAME.local:11311/"
else # We are running on the Duckiebot, which can always reach itself
    export ROS_MASTER_URI="http://localhost:11311/"
fi

export DUCKIEFLEET_ROOT=/data/config
export VEHICLE_NAME=$HOSTNAME


