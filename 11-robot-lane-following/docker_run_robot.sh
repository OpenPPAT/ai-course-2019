#!/bin/bash

docker run -it --rm \
       --runtime nvidia \
       --network=host \
       -v ${PWD}:/ai-course-2019 \
       -w /ai-course-2019/11-robot-lane-following \
       argnctu/ai-course:arm64v8-pytorch-ros
       start_nano.sh
