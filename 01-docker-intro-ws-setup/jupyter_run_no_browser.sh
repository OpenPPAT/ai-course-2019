#!/usr/bin/env sh
RED='\033[0;31m'
NC='\033[0m'

if [ ! -z "${JUPYTER_PORT}" ] ; then
    port="${JUPYTER_PORT}"
else
    port="8888"
fi

# Check the defalt network port is available. If not, change it
ret_code=$(netstat -tulnp | grep :$port)
cnt=1;
while [[ ! -z $ret_code && cnt -le 8 ]]
do 
    echo -e "${RED}PORT:$port is already in use, trying another port.${NC}"
    port=$((port + 1))
    cnt=$((cnt + 1))
    # ret_code="$(lsof -Pi :$port -sTCP:LISTEN -t)"
    ret_code=$(netstat -tulnp | grep :$port)
    sleep 0.1
done

#Don't open the broswer automatically
# Allow root because docker container is runnig as 'root'
jupyter notebook --no-browser \
                --allow-root \
                --ip="0.0.0.0" \
                --port="$port"    
