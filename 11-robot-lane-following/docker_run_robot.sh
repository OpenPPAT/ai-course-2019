#!/bin/bash

docker run -it --rm \
       --name=imitation-node \
       --runtime nvidia \
       --network=host \
       -v ${PWD}/..:/ai-course-2019 \
       -w /ai-course-2019/11-robot-lane-following/nano_ws \
       argnctu/ai-course:arm64v8-pytorch-ros #/ai-course-2019/11-robot-lane-following/start_nano.sh
