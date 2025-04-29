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

# Clone Livox ROS2 driver - Use the stable version that works with ROS2 Humble
RUN git clone https://github.com/Livox-SDK/livox_ros2_driver.git src/livox_ros2_driver

# Instead of using the SDK inside the driver, build and install it separately
WORKDIR /tmp
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git

# Add missing include to Livox SDK
WORKDIR /tmp/Livox-SDK/sdk_core/src/base
RUN sed -i '/#include "noncopyable.h"/a #include <memory>' thread_base.h

# Build and install Livox SDK
WORKDIR /tmp/Livox-SDK
RUN rm -rf build && mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    make install

# Make sure the installed library is found
RUN ldconfig

# Now modify the CMake files in the ROS2 driver to find the SDK we just installed
WORKDIR /ros2_ws/src/livox_ros2_driver
# Remove the SDK that comes with the driver
RUN rm -rf livox_sdk_vendor

# Rather than modifying CMakeLists.txt, let's create a proper symbolic link to the system SDK
RUN mkdir -p livox_sdk_vendor && \
    ln -s /usr/local/lib/liblivox_sdk_static.a livox_sdk_vendor/liblivox_sdk_static.a && \
    ln -s /usr/local/include/livox_sdk.h livox_sdk_vendor/livox_sdk.h && \
    ln -s /usr/local/include/livox_def.h livox_sdk_vendor/livox_def.h && \
    mkdir -p livox_sdk_vendor/include && \
    mkdir -p livox_sdk_vendor/lib && \
    ln -s /usr/local/include livox_sdk_vendor/include/livox_sdk && \
    ln -s /usr/local/lib/liblivox_sdk_static.a livox_sdk_vendor/lib/liblivox_sdk_static.a

# Copy config file
COPY config/livox_lidar_config.json /ros2_ws/src/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json

# Build workspace
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Add entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]