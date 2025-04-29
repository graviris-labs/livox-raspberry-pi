#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
source /ros2_ws/install/setup.bash

# Run livox_ros2_driver node
ros2 run livox_ros2_driver livox_ros2_driver_node --ros-args -p auto_connect:=false -p user_config_path:=/ros2_ws/src/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json -p cmdline_input_bd_code:=""
