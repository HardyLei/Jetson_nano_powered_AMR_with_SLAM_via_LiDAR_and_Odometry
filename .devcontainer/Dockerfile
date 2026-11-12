FROM ros:humble

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    cmake \
    build-essential \
    git \
    vim \
    nano \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src/YDLidar-SDK /tmp/YDLidar-SDK

RUN cmake -S /tmp/YDLidar-SDK -B /tmp/YDLidar-SDK/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TEST=OFF \
    && cmake --build /tmp/YDLidar-SDK/build --parallel "$(nproc)" \
    && cmake --install /tmp/YDLidar-SDK/build \
    && rm -rf /tmp/YDLidar-SDK

COPY scripts/YbImuLib /tmp/YbImuLib

RUN python3 -m pip install --no-cache-dir \
    pyserial \
    smbus2 \
    /tmp/YbImuLib \
    && rm -rf /tmp/YbImuLib

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /workspace