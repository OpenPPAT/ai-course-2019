#!/bin/bash
docker run -it --rm --net=host \
            --name=ai-course-lite \
            -v ${HOME}/ai-course-2019:/home/student/ai-course-2019 \
            -w /home/student/ai-course-2019/05-jupyter-Fashion-Mnist-dataset-pytorch \
            argnctu/ai-course:jupyter-05

