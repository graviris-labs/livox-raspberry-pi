#!/bin/bash
set -e

# Source ROS1 environment
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
else
    echo "ERROR: /opt/ros/noetic/setup.bash not found!"
    exit 1
fi

# Source workspace
if [ -f "/home/livox/catkin_ws/devel/setup.bash" ]; then
    source /home/livox/catkin_ws/devel/setup.bash
else
    echo "Workspace not built, attempting to build..."
    cd /home/livox/catkin_ws && catkin_make
    source /home/livox/catkin_ws/devel/setup.bash
fi

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /home/livox/catkin_ws/devel/setup.bash" >> ~/.bashrc
exec bash

# Launch driver
exec roslaunch livox_ros_driver2 msg_MID360.launch
