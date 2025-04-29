FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Set up locales
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install basic utilities
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

# Add ROS2 Humble repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions

# Fix empy for ROS2
RUN pip3 install empy==3.3.4

# Setup ROS2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Clone Livox ROS2 Driver 2
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2
WORKDIR /ros2_ws/src/livox_ros_driver2
RUN git checkout ros2  # ✅ VERY important! Livox Driver2 requires ros2 branch!

# Copy your config
WORKDIR /ros2_ws
COPY config/livox_lidar_config.json src/livox_ros_driver2/config/livox_lidar_config.json

# Build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
