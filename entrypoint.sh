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

grep -qxF 'source /opt/ros/noetic/setup.bash' ~/.bashrc || echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
grep -qxF 'source /home/livox/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source /home/livox/catkin_ws/devel/setup.bash' >> ~/.bashrc

export ROS_MASTER_URI=http://192.168.50.191:11311
export ROS_HOSTNAME=192.168.50.191

# Launch driver
exec roslaunch livox_ros_driver2 msg_MID360.launch
