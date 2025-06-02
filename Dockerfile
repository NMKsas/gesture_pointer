# Setting the environment on Ubuntu 20.04 (Focal Fossa)

FROM ros:noetic-ros-base-focal


RUN apt-get update || true && apt-get install -y curl && \
    # avoiding GPG error 
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \ 
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list && \
    curl -sS https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-catkin \
    pip && \
    rosdep update && \
    apt-get dist-upgrade -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update && apt-get upgrade 
RUN mkdir -p /up/ros1env/src/
COPY /json_files /up/ros1env/example_files
COPY requirements.txt /up/ros1env
