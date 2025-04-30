#!/bin/bash
set -e

# Set ROS networking vars (match docker-compose)
export ROS_MASTER_URI=http://192.168.50.191:11311
export ROS_HOSTNAME=192.168.50.49
export ROS_IP=192.168.50.49

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_HOSTNAME=$ROS_HOSTNAME"
echo "ROS_IP=$ROS_IP"

# Source ROS setup
source /opt/ros/noetic/setup.bash
source /home/livox/catkin_ws/devel/setup.bash

# Wait for ROS master to be reachable
echo "Testing connection to ROS master..."
until curl --output /dev/null --silent --head --fail $ROS_MASTER_URI; do
    echo "Failed to connect to ROS master"
    sleep 2
done

# Start listener
exec python3 /home/livox/lidar_sqlite_listener.py
