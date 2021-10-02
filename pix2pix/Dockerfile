FROM osrf/ros:melodic-desktop-full
# Ubuntu melodic
# MAINTAINER sktometometo <shinjo@jsk.imi.i.u-tokyo.ac.jp>
MAINTAINER MiyabiTane <tanemoto@jsk.imi.i.u-tokyo.ac.jp>

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       apt-utils \
       sudo \
       vim \
       tmux \
       git \
    && apt-get -y clean \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -g 1000 developer && \
    useradd  -g      developer -G sudo -m -s /bin/bash ubuntu && \
    echo 'ubuntu:ubuntu' | chpasswd

RUN echo 'Defaults visiblepw'            >> /etc/sudoers
RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# install tensorflow
RUN sudo apt update
RUN sudo apt install -y python3-dev python3-pip python3-venv
RUN pip3 install -U pip
RUN pip3 install --upgrade tensorflow==2.6.0
RUN pip3 install matplotlib
RUN pip3 install opencv-contrib-python
RUN pip3 install opencv-python

COPY /training_checkpoints /deco_tensor/training_checkpoints
COPY trained_pix2pix.py /deco_tensor/trained_pix2pix.py
COPY make_deco_with_GA.py /deco_tensor/make_deco_with_GA.py
COPY share /deco_tensor/share
WORKDIR /deco_tensor 
