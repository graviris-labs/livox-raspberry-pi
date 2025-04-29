FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Set up locales
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install base utilities
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    wget \
    git \
    build-essential \
    python3-pip \
    libpcl-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    cmake \
    net-tools

# Add ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update

# Install ROS2 and ROS-specific packages
RUN apt-get install -y \
    ros-humble-ros-base \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions

# Fix empy bug for ROS2
RUN pip3 uninstall -y empy && pip3 install empy==3.3.4

# Setup ROS2 env
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Clone Livox ROS2 driver
RUN git clone https://github.com/Livox-SDK/livox_ros2_driver.git src/livox_ros2_driver

# Upgrade Livox SDK inside driver
WORKDIR /ros2_ws/src/livox_ros2_driver
RUN rm -rf livox_sdk_vendor
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git livox_sdk_vendor

# FIX: Add missing include to thread_base.h file
# This is done directly using echo to add the include line after noncopyable.h
WORKDIR /ros2_ws/src/livox_ros2_driver/livox_sdk_vendor/sdk_core/src/base
RUN cp thread_base.h thread_base.h.backup && \
    grep -B 1000 -m 1 "#include \"noncopyable.h\"" thread_base.h > thread_base.h.top && \
    echo "#include <memory>" > thread_base.h.include && \
    grep -A 1000 "#include \"noncopyable.h\"" thread_base.h > thread_base.h.bottom && \
    cat thread_base.h.top thread_base.h.include thread_base.h.bottom > thread_base.h && \
    rm thread_base.h.top thread_base.h.include thread_base.h.bottom

# Also add missing include to thread_base.cpp file just to be safe
WORKDIR /ros2_ws/src/livox_ros2_driver/livox_sdk_vendor/sdk_core/src/base
RUN cp thread_base.cpp thread_base.cpp.backup && \
    grep -B 1000 -m 1 "#include <thread>" thread_base.cpp > thread_base.cpp.top && \
    echo "#include <memory>" > thread_base.cpp.include && \
    grep -A 1000 "#include <thread>" thread_base.cpp > thread_base.cpp.bottom && \
    cat thread_base.cpp.top thread_base.cpp.include thread_base.cpp.bottom > thread_base.cpp && \
    rm thread_base.cpp.top thread_base.cpp.include thread_base.cpp.bottom

# Return to workspace
WORKDIR /ros2_ws

# Copy config file
COPY config/livox_lidar_config.json /ros2_ws/src/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Add entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]