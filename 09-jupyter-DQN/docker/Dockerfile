FROM dorowu/ubuntu-desktop-lxde-vnc

RUN apt-get update && \
    apt-get install -y \
    net-tools \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    gedit \
    python3-pip \
    python-opengl \
    libsm6 \
    sudo \
    build-essential \
    vim \
    swig \
    git \
    libxext6 \
    libxrender-dev\
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    pygame \
    tensorflow==1.14 \
    matplotlib \
    jupyter \
    Box2D \
    box2d \
    box2d-kengz \
    box2d-py \
    gym \
    gym[box2d] \
    Pillow \
    opencv-python \
    numpy==1.16 \
    gast==0.2.2

# RUN useradd -m flappy && echo "flappy:aicourse" | chpasswd && adduser flappy sudo
# USER flappy