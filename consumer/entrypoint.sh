#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /home/livox/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.50.191:11311
export ROS_HOSTNAME=192.168.50.49

python3 /home/livox/lidar_sqlite_listener.py
