FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create ROS workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Clone Livox ROS2 driver (use master branch, NOT tag)
WORKDIR /ros2_ws/src
RUN git clone https://github.com/Livox-SDK/livox_ros2_driver.git

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Create directories
RUN mkdir -p /data

# Set up entrypoint
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Make sure ROS 2 environment is sourced in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

# Set default CMD
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run livox_ros2_driver livox_ros2_driver_node --ros-args -p auto_connect:=false -p user_config_path:=/config/livox_lidar_config.json"]
