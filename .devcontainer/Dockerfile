FROM ros:humble

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    vim \
    nano \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

COPY scripts/YbImuLib /tmp/YbImuLib

RUN python3 -m pip install --no-cache-dir \
    pyserial \
    smbus2 \
    /tmp/YbImuLib \
    && rm -rf /tmp/YbImuLib

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /workspace