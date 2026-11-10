FROM ros:humble

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    git \
    vim \
    nano \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /workspace