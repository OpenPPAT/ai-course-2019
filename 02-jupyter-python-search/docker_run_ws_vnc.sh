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



########### Other instructions to check whether the default network port are occupied or not ###########
RED='\033[0;31m'
GREEN='\033[0;32m'
COLOR_YELLOW='\033[0;33m'
NC='\033[0m'

host_vnc_port=6901
host_jupyter_port=8888

ret_code=$(netstat -tuln | grep :$host_vnc_port)
cnt=1;
while [[ ! -z $ret_code && cnt -le 10 ]]
do 
    echo -e "${RED}VNC_PORT:$host_vnc_port is already in use, trying another port.${NC}"
    host_vnc_port=$((host_vnc_port + 1))
    cnt=$((cnt + 1))
    # ret_code="$(lsof -Pi :$port -sTCP:LISTEN -t)"
    ret_code=$(netstat -tuln | grep :$host_vnc_port)
    sleep 0.1
done

ret_code=$(netstat -tuln | grep :$host_jupyter_port)
cnt=1;
while [[ ! -z $ret_code && cnt -le 10 ]]
do 
    echo -e "${RED}JUPYTER_PORT:$host_jupyter_port is already in use, trying another port.${NC}"
    host_jupyter_port=$((host_jupyter_port + 1))
    cnt=$((cnt + 1))
    # ret_code="$(lsof -Pi :$port -sTCP:LISTEN -t)"
    ret_code=$(netstat -tuln | grep :$host_jupyter_port)
    sleep 0.1
done

echo -e "Use the port: ${GREEN}${host_vnc_port} as VNC_PORT.${NC}"
echo -e "Use the port: ${GREEN}${host_jupyter_port} as JUPYTER_PORT.${NC}"

nvidia-docker run -it --rm --name="ai-course-vnc${host_vnc_port}" \
            -p ${host_vnc_port}:6901 \
            -p ${host_jupyter_port}:8888 \
            -v ${HOME}/ai-course-2019:/home/student/ai-course-2019 \
            -w /home/student/ai-course-2019/02-jupyter-python-search \
            argnctu/ai-course:ws-vnc bash