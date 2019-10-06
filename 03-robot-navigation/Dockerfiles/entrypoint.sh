#!/bin/bash
source /home/${USER}/ai-course-2019/03-robot-navigation/robot_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.50.150:11311

export ROS_IP=$(echo $(hostname -I) | cut -d' ' -f1)
roslaunch diff_drive joystick.launch
