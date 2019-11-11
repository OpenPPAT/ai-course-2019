FROM pytorch/pytorch:1.2-cuda10.0-cudnn7-runtime

MAINTAINER Biomotion

EXPOSE 8888

ARG PYTHON_VERSION=3.7
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    unzip \
    vim && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip install gdown jupyter torchvision numpy matplotlib && \
    rm -rf /root/.cache/pip

RUN jupyter notebook --generate-config && \
    echo "c.NotebookApp.allow_root = True" >> /root/.jupyter/jupyter_notebook_config.py && \
    echo "c.NotebookApp.ip = '0.0.0.0'" >> /root/.jupyter/jupyter_notebook_config.py

#ENV USER=user
#ENV PASSWD=user
#ENV UID=1000
#ENV GID=1000

#RUN useradd -mU -s /bin/bash -p $PASSWD $USER && \
#    usermod --uid $UID $USER && \
#    groupmod --gid $GID $USER && \
#    chsh -s /bin/bash $USER
    
#USER $USER
#ENV  HOME=/home/$USER
#WORKDIR /home/$USER


CMD ["bash"]
