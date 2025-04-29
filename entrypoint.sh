#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
source /ros2_ws/install/setup.bash

# Run Livox launch (this is the real way)
ros2 launch livox_ros_driver2 mid360.launch.py param_file:=/ros2_ws/src/livox_ros_driver2/config/livox_lidar_config.json
