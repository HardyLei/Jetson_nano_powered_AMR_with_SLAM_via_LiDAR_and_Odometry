FROM ros:humble

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake git nano passwd procps psmisc qtbase5-dev \
    python3-colcon-common-extensions python3-pip python3-pytest python3-yaml \
    ros-humble-ament-lint-common ros-humble-ament-mypy \
    ros-humble-imu-filter-madgwick ros-humble-rviz-imu-plugin ros-humble-rviz2 \
    ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher ros-humble-tf2-tools \
    ros-humble-teleop-twist-keyboard ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

COPY .devcontainer/explore-start-paused.patch .devcontainer/explore-continue-mapping.patch /tmp/
ARG AMR_EXPLORE_REV=326cf8a0b487c34246bb8f3326afbcd69576dc60
RUN git init /tmp/amr-explore/src/m-explore-ros2 \
    && git -C /tmp/amr-explore/src/m-explore-ros2 fetch --depth=1 \
        https://github.com/robo-friends/m-explore-ros2.git "$AMR_EXPLORE_REV" \
    && git -C /tmp/amr-explore/src/m-explore-ros2 checkout --detach FETCH_HEAD \
    && git -C /tmp/amr-explore/src/m-explore-ros2 apply /tmp/explore-start-paused.patch \
    && git -C /tmp/amr-explore/src/m-explore-ros2 apply /tmp/explore-continue-mapping.patch \
    && bash -c 'source /opt/ros/humble/setup.bash && cd /tmp/amr-explore \
        && MAKEFLAGS=-j2 colcon build --executor sequential --merge-install \
            --install-base /opt/amr-explore --packages-up-to explore_lite \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF' \
    && rm -rf /tmp/amr-explore /tmp/explore-start-paused.patch /tmp/explore-continue-mapping.patch

COPY .devcontainer/slam-gmapping-humble.patch /tmp/
RUN git init /tmp/amr-gmapping/src/slam_gmapping \
    && git -C /tmp/amr-gmapping/src/slam_gmapping fetch --depth=1 \
        https://github.com/GMHadou/slam_gmapping_Humble.git 7ffe2f768bcd75b5158a41937fcc2abd33173143 \
    && git -C /tmp/amr-gmapping/src/slam_gmapping checkout --detach FETCH_HEAD \
    && git -C /tmp/amr-gmapping/src/slam_gmapping apply /tmp/slam-gmapping-humble.patch \
    && bash -c 'source /opt/ros/humble/setup.bash && cd /tmp/amr-gmapping \
        && MAKEFLAGS=-j2 colcon build --executor sequential --merge-install \
            --install-base /opt/amr-gmapping --packages-up-to slam_gmapping \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_TESTING=OFF' \
    && rm -rf /tmp/amr-gmapping /tmp/slam-gmapping-humble.patch

COPY ros2_ws/src/YDLidar-SDK /tmp/ros-src/YDLidar-SDK
ARG CMAKE_BUILD_PARALLEL_LEVEL=1
RUN sdk=/tmp/ros-src/YDLidar-SDK \
    && if [ ! -f "$sdk/CMakeLists.txt" ]; then \
        git init "$sdk" \
        && git -C "$sdk" fetch --depth=1 https://github.com/HardyLei/YDLidar-SDK.git 12216f834ebd83b1020f8d3f873064e7a17d34dd \
        && git -C "$sdk" checkout --detach FETCH_HEAD; \
    fi \
    && cmake -S "$sdk" -B /tmp/ydlidar-build -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF \
    && cmake --build /tmp/ydlidar-build --parallel "$CMAKE_BUILD_PARALLEL_LEVEL" \
    && cmake --install /tmp/ydlidar-build \
    && ldconfig \
    && rm -rf /tmp/ros-src /tmp/ydlidar-build

COPY scripts/YbImuLib /tmp/YbImuLib
RUN python3 -m pip install --no-cache-dir pyserial==3.5 smbus2==0.5.0 /tmp/YbImuLib \
    && python3 -c 'import YbImuLib' \
    && rm -rf /tmp/YbImuLib

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'source /opt/amr-explore/local_setup.bash' \
    'source /opt/amr-gmapping/local_setup.bash' \
    'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' \
    >> /etc/bash.bashrc

WORKDIR /workspace
