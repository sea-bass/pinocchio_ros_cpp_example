FROM ros:humble as base

RUN apt update && apt install -y ros-humble-pinocchio

RUN mkdir -p /workspace/src
WORKDIR /workspace

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
