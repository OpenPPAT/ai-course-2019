#!/bin/bash

docker_info=$(docker info)
if [[ $docker_info == *"nvidia"* ]]; then
        runtime=nvidia-docker
else
        runtime=docker
fi


$runtime run -it --rm --net=host \
            -v ${HOME}/ai-course-2019:/home/student/ai-course-2019 \
            -w /home/student/ai-course-2019/06-jupyter-cnn \
            argnctu/ai-course:pytorch-jupyter
