FROM dorowu/ubuntu-desktop-lxde-vnc

RUN apt-get update && \
    apt-get install -y \
    net-tools \
    apt-utils \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    gedit \
    python3-pip \
    libsm6 \
    sudo \
    build-essential \
    vim \
    git \
    libxext6 \
    libxrender-dev\
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    pygame \
    tensorflow==1.14 \
    matplotlib \
    jupyter \
    Pillow \
    opencv-python \
    numpy==1.16

# RUN useradd -m flappy && echo "flappy:aicourse" | chpasswd && adduser flappy sudo
# USER flappy