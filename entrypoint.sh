#!/bin/bash
set -e

# Source ROS1 env and workspace
source /opt/ros/noetic/setup.bash
source /home/livox/catkin_ws/devel/setup.bash

# Run the driver with the config
exec roslaunch livox_ros_driver2 msg_MID360.launch
