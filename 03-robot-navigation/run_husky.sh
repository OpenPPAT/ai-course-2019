#!/usr/bin/env bash

source robot_ws/devel/setup.bash

echo "Setting ROS_MASTER_URI..."
if [ $# -gt 0 ]; then
    # provided a hostname, use it as ROS_MASTER_URI
    export ROS_MASTER_URI=http://$1.local:11311/
else
    [ -z "$HOSTNAME"        ] && { echo -e "\n\nThe variable HOSTNAME is not set. I need this info for setting up ROS. \n\n\n\n"; return 2;       }
    echo "No hostname provided. Using $HOSTNAME."
    export ROS_MASTER_URI=http://$HOSTNAME.local:11311/
fi
echo "ROS_MASTER_URI set to $ROS_MASTER_URI"
