#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /home/livox/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.50.191:11311
export ROS_HOSTNAME=192.168.50.49  # Use your Mac's IP address

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_HOSTNAME=$ROS_HOSTNAME"

# Test connection to ROS master
echo "Testing connection to ROS master..."
timeout 10 rostopic list || echo "Failed to connect to ROS master"

# Run the listener
python3 /home/livox/lidar_sqlite_listener.py
