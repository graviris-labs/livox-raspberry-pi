#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Debug information
echo "Checking ROS2 packages..."
ros2 pkg list | grep livox

echo "Checking if workspace was built properly..."
ls -la /ros2_ws/install

# Source the workspace (if it exists)
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
    echo "Workspace sourced successfully"
else
    echo "ERROR: Workspace setup.bash not found"
    # Try to build the workspace
    echo "Trying to build the workspace..."
    cd /ros2_ws
    colcon build --symlink-install
    
    if [ -f "/ros2_ws/install/setup.bash" ]; then
        source /ros2_ws/install/setup.bash
        echo "Workspace built and sourced successfully"
    else
        echo "ERROR: Failed to build workspace"
        exit 1
    fi
fi

# Check if the package exists now
echo "Checking ROS2 packages after sourcing workspace..."
ros2 pkg list | grep livox

# Check launch files
echo "Available launch files:"
find /ros2_ws -name "*.py" | grep launch

# Run Livox driver
echo "Running Livox driver..."
cd /ros2_ws
ros2 launch livox_ros_driver2 rviz_MID360_launch.py