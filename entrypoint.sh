#!/bin/bash
set -e
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Run Livox driver specifically for Mid-360
ros2 launch livox_ros_driver2 rviz_MID360_launch.py