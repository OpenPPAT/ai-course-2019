#
# docker run [OPTIONS] IMAGE[:TAG] [COMMAND]
#

# OPTIONS:
# -i (--interactive):       Keep STDIN (standard input) open even if not attached.
# -t (--tty):               Allocate a pseudo-tty (virtual terminal).
# -p (--publish):           Publish a container’s network port(s) to the host, usage: "host_port:container_port".
# -v (--volume):            Bind mount a volume,  usage: "host_directory:container_directory".
# --rm:                     Automatically remove the container when it exits.

# IMAGE:                    Docker image
# TAG:                      Tags for identifying different version of docker images.

# COMMAND:                  The command once enter the container. If you don't assign any, it will run the default command which is set in dockerfile.  

nvidia-docker run -it --rm \
            -p 5901:5901 \
            -p 6901:6901 \
            -p 8888:8888 \
            -v ${HOME}/ai-course-2019:/home/student/ai-course-2019 \
            argnctu/ai-course:ws-vnc