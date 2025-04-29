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

# Install colcon
RUN pip3 install -U colcon-common-extensions

# Fix empy bug for ROS2
RUN pip3 install empy==3.3.4

# Add ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp

# Setup ROS2 env
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Clone the livox_ros_driver2 which is more compatible with ROS2 Humble
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2

# Copy config file
COPY config/livox_lidar_config.json /ros2_ws/src/livox_ros_driver2/config/

# Build the ROS2 driver
WORKDIR /ros2_ws/src/livox_ros_driver2
RUN chmod +x build.sh
RUN bash ./build.sh humble

# Copy and run the fix_thread_base script if needed
COPY fix_thread_base.sh /fix_thread_base.sh
RUN chmod +x /fix_thread_base.sh
# RUN /fix_thread_base.sh

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
