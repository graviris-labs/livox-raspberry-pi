#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /home/rosuser/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.50.191:11311  # Pi’s IP (ROS master)
export ROS_HOSTNAME=192.168.50.49                  # Mac’s IP

exec rosrun lidar_sqlite_listener lidar_sqlite_listener.py

