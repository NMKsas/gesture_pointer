# Setting the environment on Ubuntu 20.04 (Focal Fossa)

FROM ros:noetic-ros-base-focal

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-catkin && \
    rosdep update && \
    apt-get dist-upgrade -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update && apt-get upgrade 

RUN mkdir -p /up/ros1env/src/
COPY /gesture_pointer /up/ros1env/src/gesture_pointer
COPY /snap_to_target /up/ros1env/src/snap_to_target
COPY /json_files /up/ros1env/example_files
