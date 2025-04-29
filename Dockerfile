FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Locale setup
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install base tools
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
    cmake

# Add ROS 2 APT repo + keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update

# Install ROS 2 Humble and colcon
RUN apt-get install -y \
    ros-humble-ros-base \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions

# Fix empy for ROS interface generation
RUN pip3 uninstall -y empy && pip3 install empy==3.3.4

# Setup ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create workspace and clone Livox driver
WORKDIR /ros2_ws
RUN mkdir -p src
RUN git clone https://github.com/Livox-SDK/livox_ros2_driver.git src/livox_ros2_driver

# Copy your config
COPY config/livox_lidar_config.json /ros2_ws/src/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Add entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
