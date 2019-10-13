FROM arm32v7/ros:melodic-perception
# FROM arm64v8/ubuntu:bionic

SHELL ["/bin/bash","-c"]

# ARG DEBIAN_FRONTEND=noninteractive
# RUN apt-get update && apt-get install -y --no-install-recommends make g++
# # Prerequisite linux tool for ROS 
# RUN apt-get update && \
#     apt-get install -y wget lsb-release iputils-ping \
#                        build-essential vim htop sshfs \
#                        nfs-common git net-tools && \
#     rm -rf /var/lib/apt/lists/*
ENV ROS_DISTRO melodic
RUN apt-get update && apt-get install -y sudo python-pip \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-teb-local-planner-tutorials \
    ros-${ROS_DISTRO}-stage-ros \
    ros-${ROS_DISTRO}-teleop-twist-joy

RUN pip install simple_pid \
    Adafruit_GPIO



# Create a new user
ENV USER ai_student
RUN useradd -m -d /home/${USER} ${USER} -p "" \
 && chown -R ${USER} /home/${USER} /root/ && adduser ${USER} sudo
USER ${USER}

RUN sudo rm /etc/ros/rosdep/sources.list.d/20-default.list && \
    sudo rosdep init && rosdep update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER}/.bashrc && \
    source /home/${USER}/.bashrc
    
RUN cd /home/${USER} && \
    source /opt/ros/${ROS_DISTRO}/setup.sh && \
    git clone https://github.com/openppat/ai-course-2019 && \
    cd ai-course-2019/03-robot-navigation/robot_ws && \
    catkin_make

RUN source /home/${USER}/ai-course-2019/03-robot-navigation/robot_ws/devel/setup.bash

# EXPOSE 8888

# CMD ["bash"]
COPY --chown=1000 ./entrypoint.sh /home/${USER}/entrypoint.sh
# Set excutable permission to the file
RUN chmod +x /home/${USER}/entrypoint.sh
CMD ["/bin/bash", "-c", "/home/${USER}/entrypoint.sh", "&& /bin/bash" ]

