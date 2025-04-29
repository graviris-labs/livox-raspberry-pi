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
    net-tools \
    vim \
    libapr1-dev

# Add ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions

# Fix empy bug
RUN pip3 install empy==3.3.4

# Setup ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Clone Livox ROS2 Driver
RUN git clone https://github.com/Livox-SDK/livox_ros2_driver.git src/livox_ros2_driver

# Patch CMakeLists.txt inside the nested directory to link livox_sdk_static
RUN sed -i '/find_package(PCL REQUIRED)/a link_libraries(livox_sdk_static)' /ros2_ws/src/livox_ros2_driver/livox_ros2_driver/CMakeLists.txt

# Replace SDK
WORKDIR /ros2_ws/src/livox_ros2_driver
RUN rm -rf livox_sdk_vendor
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git livox_sdk_vendor

# Patch missing <memory> includes
RUN sed -i '1i#include <memory>' /ros2_ws/src/livox_ros2_driver/livox_sdk_vendor/sdk_core/src/base/thread_base.h
RUN sed -i '1i#include <memory>' /ros2_ws/src/livox_ros2_driver/livox_sdk_vendor/sdk_core/src/base/thread_base.cpp

# Build and install Livox SDK statically with -fPIC
WORKDIR /ros2_ws/src/livox_ros2_driver/livox_sdk_vendor
RUN mkdir -p build && cd build && cmake .. -DBUILD_STATIC=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON && make -j4 && make install

# Create fake livox_sdkConfig.cmake for cmake to find livox_sdk
RUN mkdir -p /usr/local/lib/cmake/livox_sdk && \
    echo 'include_directories(/usr/local/include)' > /usr/local/lib/cmake/livox_sdk/livox_sdkConfig.cmake && \
    echo 'link_directories(/usr/local/lib)' >> /usr/local/lib/cmake/livox_sdk/livox_sdkConfig.cmake

# Go back to workspace
WORKDIR /ros2_ws

# Copy your lidar config
COPY config/livox_lidar_config.json src/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json

# Build the full ROS2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=/usr/local"

# Copy and setup entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
