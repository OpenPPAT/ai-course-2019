# pozyx_ros

ROS node and test application for POZYX

reference:

https://pypozyx.readthedocs.io/en/develop/index.html

https://github.com/pozyxLabs/Pozyx-Python-library

## How to run pozyx_ros node

install the pypozyx package:
  
    sudo pip install pypozyx requests
    
launch pozyx node:

    roslaunch pozyx_ros pozyx_node.launch

A PoseStamped message will then published to **/pozyx_node/pozyx_pose**
