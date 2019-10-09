#!/usr/bin/env bash
if [ $# -gt 0 ]; then
    if [ "$1" == "same" ]; then
        docker exec -it ai-course-laptop bash
    fi
else
        docker run -it --net=host --rm --name=ai-course-laptop \
                    -v $(pwd):/home/student/ai-course-2019/03-robot-navigation \
                    -w /home/student/ai-course-2019/03-robot-navigation \
                    argnctu/ai-course:ros-jupyter-lite
fi