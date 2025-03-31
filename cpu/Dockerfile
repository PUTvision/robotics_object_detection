FROM osrf/ros:jazzy-desktop

# SETUP ENVS
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


RUN apt update && apt install -y \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-rqt-image-view \
    python3-full \
    python3-pip \
    python3-opencv \
    nano vim \
    wget \
    git \
    curl \
    zip unzip

ENV ROS_DISTRO jazzy
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

RUN mkdir -p /robotics1/src


WORKDIR /robotics1

RUN mkdir tmp/
RUN git clone https://github.com/PUTvision/robotics_object_detection.git tmp
RUN mv tmp/robotics_object_detection/part_1_training model_training
RUN mv tmp/robotics_object_detection/part_2_deploy src/robotics_object_detection
RUN rm -rf tmp

RUN pip3 install -r model_training/requirements.txt --break-system-packages


RUN rm -rf /var/lib/apt/lists/*
