#!/bin/bash

catkin_make -C nano_ws

export ROS_MASTER_URI=http://10.42.0.2:11311

source nano_ws/devel/setup.bash
