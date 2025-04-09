ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO} as base

RUN apt update && \
    apt upgrade -y && \
    apt install -y ros-${ROS_DISTRO}-pinocchio

RUN mkdir -p /workspace/src
WORKDIR /workspace

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
