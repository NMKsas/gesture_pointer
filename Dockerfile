# Setting the environment on Ubuntu 22.04 (Jammy)
FROM ros:humble

ENV YOLO_CONFIG_DIR="/tmp"
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    ros-${ROS_DISTRO}-rqt* && \
    apt-get dist-upgrade -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /up/ros2env/src/
COPY /json_files /up/ros2env/example_files
COPY requirements.txt /up/ros2env
WORKDIR /up/ros2env

# Install python dependencies 
RUN pip3 install -r requirements.txt
RUN rm requirements.txt
