FROM nvcr.io/nvidia/l4t-base:r32.2

MAINTAINER benson

# setup timezone & install packages
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y tzdata dirmngr gnupg2 python-pip && \
    apt-get install -y --no-install-recommends make g++ && \
    apt-get install -y libusb-1.0 gtk+-3.0 build-essential libglfw3-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# setup keys and source.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO melodic
RUN apt-get update && apt-get install -y \ 
    ros-${ROS_DISTRO}-ros-core=1.4.1-0* \
    ros-${ROS_DISTRO}-perception=1.4.1-0* \
    ros-${ROS_DISTRO}-rgbd-launch \
    ros-${ROS_DISTRO}-tf-conversions \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

# installing pytorch
WORKDIR /tmp
RUN wget https://nvidia.box.com/shared/static/6t52xry4x2i634h1cfqvc9oaoqfzrcnq.whl -O torch-1.3.0-cp27-cp27mu-linux_aarch64.whl && \
    pip install torch-1.3.0-cp27-cp27mu-linux_aarch64.whl torchvision && \
    rm -rf torch-1.3.0-cp27-cp27mu-linux_aarch64.whl

# set the version of the realsense library
ENV LIBREALSENSE_VERSION 2.16.1
#ENV LIBREALSENSE_ROS_VERSION 2.1.0
RUN wget https://github.com/IntelRealSense/librealsense/archive/v${LIBREALSENSE_VERSION}.tar.gz && \
    tar -xvzf v${LIBREALSENSE_VERSION}.tar.gz && \
    rm v${LIBREALSENSE_VERSION}.tar.gz && \
    mkdir -p librealsense-${LIBREALSENSE_VERSION}/build && \
    cd librealsense-${LIBREALSENSE_VERSION}/build && \
    cmake .. && \
    make && \
    make install && \
    rm -rf librealsense-${LIBREALSENSE_VERSION}

# installing realsense
#RUN git clone https://github.com/biomotion/installLibrealsense.git
#WORKDIR /tmp/installLibrealsense
#RUN ./installLibrealsense.sh

RUN echo "export ROS_MASTER_URI=http://10.42.0.1:11311" >> /root/.bashrc && \
    echo "export ROS_IP=10.42.0.2" >> /root/.bashrc && \
    echo "source /ai-course-2019/11-robot-lane-following/nano_ws/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
WORKDIR /
CMD ["bash"]
