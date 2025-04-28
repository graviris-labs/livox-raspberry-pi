FROM ros:noetic-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
WORKDIR /catkin_ws
RUN mkdir -p src

# Clone Livox ROS driver
WORKDIR /catkin_ws/src
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git

# Build
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Auto-source setup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Default bash shell
ENTRYPOINT ["bash"]
