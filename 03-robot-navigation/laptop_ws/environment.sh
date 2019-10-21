#!/usr/bin/env bash
export ROS_MASTER_URI=http://192.168.50.150:11311
export ROS_IP=($hostname -I)

source ~/ai-course-2019/03-robot-navigation/laptop_ws/devel/setup.bash
