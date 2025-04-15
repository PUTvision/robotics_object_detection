FROM nvidia/cuda:12.2.2-runtime-ubuntu22.04

# SETUP ENVS
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


RUN apt update && apt install -y \
    python3-full \
    python3-pip \
    python3-opencv \
    nano vim \
    wget \
    git \
    curl \
    zip unzip


RUN apt update && apt install software-properties-common -y
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt install -y \
        ros-dev-tools \
        ros-humble-ros-base \
        ros-humble-rqt \
        ros-humble-image-transport-plugins \
        ros-humble-rqt-image-view

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

ENV ROS_DISTRO humble
ENV ROS_AUTOMATIC_DISCOVERY_RANGE LOCALHOST
ENV ROS_LOCALHOST_ONLY 1


RUN mkdir -p /robotics1/src

WORKDIR /robotics1

RUN mkdir tmp/
RUN git clone https://github.com/PUTvision/robotics_object_detection.git tmp
RUN mv tmp/robotics_object_detection/part_1_training model_training
RUN mv tmp/robotics_object_detection/part_2_deploy src/robotics_object_detection
RUN rm -rf tmp

RUN pip3 install -r model_training/requirements.txt

RUN rm -rf /var/lib/apt/lists/*
