#!/bin/bash

catkin_make clean
catkin_make

export ROS_MASTER_URI=http://10.42.0.1:11311
source devel/setup.bash

roscore
