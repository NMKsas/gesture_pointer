FROM ros:jazzy

ENV YOLO_CONFIG_DIR="/tmp"

RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y \
    python3-pip \
    python3-venv \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-aruco-ros && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /up/ros2env/src/
COPY /json_files /up/ros2env/example_files
COPY requirements.txt /up/ros2env
WORKDIR /up/ros2env

SHELL ["/bin/bash", "-c"]

# python environment setup
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    python3 -m venv .venv --system-site-packages && \
    . .venv/bin/activate && \
    python3 -m pip install --no-cache-dir -r requirements.txt

RUN rm requirements.txt
