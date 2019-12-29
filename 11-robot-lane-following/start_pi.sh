#!/bin/bash

cd ~/duckietown
source environment.sh
source set_ros_master.sh

roslaunch duckietown joystick.launch &&
roslaunch duckietown camera.launch
