#!/bin/bash
set -e
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
# Run Livox driver
ros2 run livox_ros2_driver livox_ros2_driver_node --ros-args -p data_src:=0 -p user_config_path:=/ros2_ws/src/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json

