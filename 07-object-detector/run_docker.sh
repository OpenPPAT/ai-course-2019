#!/bin/bash
docker run -it --rm --net=host \
            -v ${HOME}/ai-course-2019:/home/student/ai-course-2019 \
            -w /home/student/ai-course-2019/07-object-detector \
            argnctu/ai-course:jupyter-07

