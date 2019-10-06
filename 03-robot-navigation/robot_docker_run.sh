#!/usr/bin/env bash

docker run -it --rm --privileged --net=host --name=super-pi \
        --device=/dev/input/js0 \
        --device=/dev/ttyUSB0 \
        --device=/dev/i2c-1 \
        --device=/dev/i2c-2 \
        --group-add="998" \
        --group-add="input" \
        -v ${HOME}/ai-course-2019/03-robot-navigation/robot_ws:/home/ai_student/robot_ws:rw \
        -w /home/ai_student/robot_ws \
        -u 1000:${UID} \
        argnctu/ai-course:super-pi