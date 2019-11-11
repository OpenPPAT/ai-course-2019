#!/bin/bash
docker run -it --rm --net=host \
            -v ${HOME}/ai-course-2019:/root/ai-course-2019 \
            -w /root/ai-course-2019/05-jupyter-Fashion-Mnist-dataset-pytorch \
            argnctu/ai-course:pytorch-jupyter


