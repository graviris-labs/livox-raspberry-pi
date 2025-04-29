FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install basic utilities
RUN apt-get update && apt-get install -y \
    locales \
    lsb-release \
    gnupg2 \
    curl \
    wget \
    git \
    sudo \
    python3-pip \
    build-essential \
    libpcl-dev \
    libboost-all-dev \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Install colcon via pip
RUN pip3 install -U colcon-common-extensions

# Fix empy package for ROS2 Humble
RUN pip3 uninstall -y empy && pip3 install empy==3.3.4

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y ros-humble-ros-base ros-humble-rmw-fastrtps-cpp ros-humble-rmw-cyclonedds-cpp \
    && apt-get clean

# Source ROS2
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create workspace
WORKDIR /ros2_ws
RUN mkdir src

# Clone Livox ROS2 Driver
RUN git clone https://github.com/Livox-SDK/livox_ros2_driver.git src/livox_ros2_driver

# Copy config file into container
COPY config/livox_lidar_config.json /ros2_ws/src/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json

# Build workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
