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
